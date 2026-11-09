// cfgstore — 플래시 섹터 두 개에 설정 기록을 이어 쓴다. 전원이 어느 순간 나가도 온전한 기록 하나는 남는다.
//   기록 = [MAGIC][seq][len][crc32(payload)][payload…(4바이트 정렬)]. 섹터 안에선 앞에서부터 차례로.
//   쓰기: 지금 섹터 끝에 덧붙인다 → 자리가 없거나 끝이 깨끗하지 않으면(찢긴 쓰기) 다른 섹터를 지우고 거기 쓴다.
//   예전 섹터는 다음에 넘어갈 때까지 지우지 않는다 = 새 기록이 다 써지기 전에도 옛 기록이 살아 있다.
//   읽기: 두 섹터에서 CRC가 맞는 기록 중 seq가 가장 큰 것.
#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const uint32_t *sec[2];       // 섹터 내용(읽기 — 메모리 매핑)
    uint32_t size;                // 섹터 크기(바이트)
    bool (*erase)(int s);
    bool (*program)(int s, uint32_t off, const uint32_t *w, uint32_t n);   // off·n은 워드 단위
} cfgstore_io_t;

uint32_t crc32_calc(const void *p, uint32_t n);
int32_t cfgstore_load(const cfgstore_io_t *io, void *payload, uint32_t len);   // seq, 없으면 -1
int32_t cfgstore_save(const cfgstore_io_t *io, const void *payload, uint32_t len);   // seq, 실패 -1
bool cfgstore_wipe(const cfgstore_io_t *io);
