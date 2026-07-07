"""음성 명령 파이프라인 — 마이크 → 발화구간검출(VAD) → Whisper STT → 인텐트 매칭.

실시간 인식 로그는 log_hub/server를 통해 브라우저(SSE)로 확인할 수 있다.
"""
