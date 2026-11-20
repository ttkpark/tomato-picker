# HITBOT Linux Python SDK (Simon Chen 공유, 2026-11-19)

출처: Google Drive 공유 폴더 "Linux Python" (https://drive.google.com/drive/folders/1PFRansOi0AZKiwpS-FhuhVm7VMw1WBRk)

- `HitbotInterface.py` — ctypes 래퍼. `CDLL('./libsmall_scara_interface.so')`로 공유 라이브러리를 열어 함수를 부른다. 통신 프로토콜은 전부 .so 안에 있다.
- `libsmall_scara_interface.so.1.0.0` — **ELF x86-64**. `file`로 확인(2026-11-19). **Jetson Orin Nano(aarch64)에서는 로드되지 않는다.**
- `API-LINUX_Python.pdf` / `API_LINUX_PY.pdf` — API 설명서(영/중). 개발 환경 표기: "win7 win10 python2/3 x86/x64". 버전 2.0.0.20200810.
- `hitbot_doc.docx` — 영문 설명서의 워드판.

결론: ARM64 빌드를 HITBOT에서 받거나, 유선 프로토콜 문서를 받아 직접 구현하거나, x86 리눅스 PC를 중계기로 두지 않으면 Jetson에서 못 쓴다.
