#!/usr/bin/env python3
# serial_bridge.py — ttyUSB0를 한 번만 열어 두고:
#   (1) 들어오는 모든 시리얼을 ~/serial_live.log 에 append (사용자가 실시간 훔쳐봄)
#   (2) ~/serial_cmd.txt 에 뭔가 써지면 그 바이트를 시리얼로 전송 후 파일 비움 (내가 명령 주입)
#   (3) USB 끊김/재인식에 견디게 자동 재오픈
# 포트를 하나만 열므로 "multiple access on port" 충돌 없음.
import serial, time, os, sys, threading

PORT = os.environ.get('BRIDGE_PORT', '/dev/ttyUSB0')
BAUD = int(sys.argv[1]) if len(sys.argv) > 1 else 9600
LOG  = os.environ.get('BRIDGE_LOG', os.path.expanduser('~/serial_live.log'))
CMD  = os.environ.get('BRIDGE_CMD', os.path.expanduser('~/serial_cmd.txt'))

def logb(b):
    with open(LOG, 'ab') as f:
        f.write(b)
        f.flush()

def log(msg):
    logb(msg.encode('utf-8'))

open(CMD, 'a').close()  # 명령파일 없으면 생성

ser = None
def open_serial():
    global ser
    while True:
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.2)
            log('\n[bridge] opened %s @ %d baud\n' % (PORT, BAUD))
            return
        except Exception as e:
            log('[bridge] open failed: %s (retry in 1s)\n' % e)
            time.sleep(1)

def cmd_watch():
    last = 0.0
    while True:
        try:
            st = os.stat(CMD)
            if st.st_mtime != last and st.st_size > 0:
                last = st.st_mtime
                data = open(CMD, 'rb').read()
                open(CMD, 'wb').close()  # 읽은 뒤 비움
                if data and ser:
                    ser.write(data)
                    log('[bridge] >>> SENT %r\n' % data)
        except Exception as e:
            log('[bridge] cmd err: %s\n' % e)
        time.sleep(0.1)

open_serial()
threading.Thread(target=cmd_watch, daemon=True).start()
log('[bridge] running. baud=%d  log=%s  cmd=%s\n' % (BAUD, LOG, CMD))

while True:
    try:
        line = ser.readline()
        if line:
            logb(line)  # 시리얼 원본 바이트 그대로 (UTF-8 한글 보존)
    except Exception as e:
        log('[bridge] read err: %s -> reopen\n' % e)
        try: ser.close()
        except Exception: pass
        open_serial()
