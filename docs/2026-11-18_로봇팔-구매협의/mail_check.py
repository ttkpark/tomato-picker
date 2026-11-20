"""새 로봇팔 견적 메일 받아오기 — 비서 서버(my_secretary) POST /chat 로 Gmail을 읽게 한다.
사용: python mail_check.py "2026-11-20 17:20"   (그 시각 이후 메일만)
비서는 첨부를 못 읽는다. 메일 발송·삭제는 사람이 한다."""
import json, sys, urllib.request
since = sys.argv[1] if len(sys.argv) > 1 else "오늘 00:00"
q = f"""(tomato-picker 로봇팔 구매 건. 사용자 지시로 확인 중입니다.)
Gmail에서 {since} 이후에 주고받은 로봇팔 견적 관련 메일을 전부 찾아 주세요. 대상: robogates.com, dhauto.kr, epson.co.kr, hitbot.cc, ufactory.cc.
- 받은 메일과 내가 보낸 메일을 시간순으로: 발신, 수신(참조 포함), 날짜·시각, 제목, 본문 원문(인용된 이전 메일·서명·고지문 제외)
- 임시보관함에만 있는 초안은 제목만 따로.
- 없으면 "새 메일 없음"이라고만. 파일로 저장하지 말고 이 답변 안에 적어 주세요. 요약하지 마세요."""
req = urllib.request.Request("http://192.168.0.178:8000/chat",
    data=json.dumps({"text": q, "session_id": "claude-code-tomato-picker"}).encode(),
    headers={"Content-Type": "application/json"})
sys.stdout.reconfigure(encoding="utf-8")
print(json.loads(urllib.request.urlopen(req, timeout=540).read())["reply"])
