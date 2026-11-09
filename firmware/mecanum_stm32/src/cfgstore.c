#include "cfgstore.h"

#define MAGIC 0x4746434Du         // "MCFG"
#define HDR_W 4u
#define ERASED 0xFFFFFFFFu

uint32_t crc32_calc(const void *p, uint32_t n) {
    const uint8_t *b = (const uint8_t *)p;
    uint32_t c = ERASED;
    while (n--) {
        c ^= *b++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

typedef struct {
    int32_t seq;                  // 이 섹터의 가장 큰 온전한 seq, 없으면 -1
    uint32_t seq_off;             // 그 기록의 워드 오프셋
    uint32_t end;                 // 덧붙일 자리(워드)
    bool clean_end;               // end 뒤가 지워진 상태인가(찢긴 쓰기가 없나)
} scan_t;

static uint32_t words_of(uint32_t len) { return (len + 3u) / 4u; }

static scan_t scan(const cfgstore_io_t *io, int s, uint32_t len) {
    scan_t r = { -1, 0, 0, false };
    const uint32_t *m = io->sec[s];
    uint32_t total = io->size / 4u, off = 0;
    while (off + HDR_W <= total) {
        if (m[off] == ERASED) break;
        if (m[off] != MAGIC || m[off + 2] > io->size || off + HDR_W + words_of(m[off + 2]) > total) {
            r.end = off; r.clean_end = false; return r;   // 찢긴 머리 — 여기 뒤로는 못 덧붙인다
        }
        uint32_t lw = words_of(m[off + 2]);
        if (m[off + 2] == len && crc32_calc(&m[off + HDR_W], len) == m[off + 3] && (int32_t)m[off + 1] > r.seq) {
            r.seq = (int32_t)m[off + 1];
            r.seq_off = off;
        }
        off += HDR_W + lw;
    }
    r.end = off;
    r.clean_end = true;
    for (uint32_t i = off; i < total; i++) if (m[i] != ERASED) { r.clean_end = false; break; }
    return r;
}

int32_t cfgstore_load(const cfgstore_io_t *io, void *payload, uint32_t len) {
    scan_t a = scan(io, 0, len), b = scan(io, 1, len);
    int s = (b.seq > a.seq) ? 1 : 0;
    scan_t *best = s ? &b : &a;
    if (best->seq < 0) return -1;
    const uint8_t *src = (const uint8_t *)&io->sec[s][best->seq_off + HDR_W];
    uint8_t *dst = (uint8_t *)payload;
    for (uint32_t i = 0; i < len; i++) dst[i] = src[i];
    return best->seq;
}

static bool write_rec(const cfgstore_io_t *io, int s, uint32_t off, int32_t seq, const void *payload, uint32_t len) {
    uint32_t pw = words_of(len);
    uint32_t buf[64];
    if (pw > 64) return false;
    for (uint32_t i = 0; i < pw; i++) buf[i] = ERASED;
    const uint8_t *src = (const uint8_t *)payload;
    uint8_t *dst = (uint8_t *)buf;
    for (uint32_t i = 0; i < len; i++) dst[i] = src[i];
    // 몸통 → crc·len·seq → MAGIC 순서. 머리(MAGIC)가 마지막이라 찢기면 '기록 없음'이거나 crc 불일치로 보인다.
    uint32_t hdr[3] = { (uint32_t)seq, len, crc32_calc(payload, len) };
    if (!io->program(s, off + HDR_W, buf, pw)) return false;
    if (!io->program(s, off + 1, hdr, 3)) return false;
    uint32_t mg = MAGIC;
    if (!io->program(s, off, &mg, 1)) return false;
    const uint32_t *m = io->sec[s];   // 다시 읽어 확인
    return m[off] == MAGIC && m[off + 3] == crc32_calc(&m[off + HDR_W], len);
}

int32_t cfgstore_save(const cfgstore_io_t *io, const void *payload, uint32_t len) {
    scan_t sc[2] = { scan(io, 0, len), scan(io, 1, len) };
    int cur = (sc[1].seq > sc[0].seq) ? 1 : 0;
    int32_t seq = (sc[cur].seq < 0 ? 0 : sc[cur].seq) + 1;
    if (seq < 0) seq = 1;
    uint32_t need = HDR_W + words_of(len);
    if (sc[cur].clean_end && sc[cur].end + need <= io->size / 4u &&
        write_rec(io, cur, sc[cur].end, seq, payload, len))
        return seq;
    int other = cur ^ 1;               // 넘어간다 — 옛 섹터는 그대로 둔다
    if (!io->erase(other)) return -1;
    return write_rec(io, other, 0, seq, payload, len) ? seq : -1;
}

bool cfgstore_wipe(const cfgstore_io_t *io) {
    return io->erase(0) && io->erase(1);
}
