# 블루투스 콘솔 — IP를 몰라도 젯슨에 들어가기

## 왜 만들었나

젯슨의 IP를 알아내려면 SSH가 필요하고, SSH를 하려면 IP를 알아야 한다. **닭과 달걀**이다.
지금까지는 같은 서브넷에서 ARP 스윕으로 찾아냈지만(`docs`의 IP 찾기 참고), 그건
**젯슨이 이미 내가 아는 망에 붙어 있을 때만** 통한다. 현장을 옮겨 새 WiFi밖에 없는데
젯슨은 아직 옛 망을 찾고 있으면, 남는 건 모니터·키보드를 물리는 것뿐이었다.

블루투스는 IP 망과 **완전히 무관한 경로**라 이 고리를 끊는다. 그리고 여기서 새 WiFi에
붙이면 된다 — **BLE 링크는 WiFi를 갈아타도 끊기지 않으므로** 결과를 그 자리에서
확인할 수 있다. (SSH로 망을 갈아타는 건 자기가 올라탄 가지를 자르는 일이라, 실패하면
복구할 방법이 없었다.)

⚠ **블루투스는 "어디서나"가 아니다 — 반경 10m다.** 건물 밖에서 들어가야 한다면
이게 아니라 Tailscale 같은 오버레이 VPN을 얹어야 한다.

## 구성

| 조각 | 무엇 |
|---|---|
| [`tools/ble_console.py`](../tools/ble_console.py) | 젯슨의 BLE 서버(Nordic UART Service) |
| [`deploy/ble-console.service`](../deploy/ble-console.service) | 부팅 자동실행 유닛 (**root** — nmcli·systemctl 때문) |
| [`android/handset/`](../android/handset/) | **안드로이드 앱**(권장). WebView 화면 + 네이티브 BLE |
| [`tools/ble_handset.html`](../tools/ble_handset.html) | 크롬 Web Bluetooth 웹앱 — ⚠ **아래 제약 참고** |

### ⚠ 웹앱이 아니라 안드로이드 앱을 쓰는 이유

Web Bluetooth로 만들어 봤지만 **샌드박스 iframe에서는 권한정책에 막힌다.**
클로드 아티팩트로 띄웠을 때 실측한 오류:

```
Failed to execute 'requestDevice' on 'Bluetooth':
Access to the feature "bluetooth" is disallowed by permissions policy.
```

호스트 페이지가 `allow="bluetooth"`를 iframe에 위임해 주지 않으면 **기기 목록조차
못 띄운다** — 앱 쪽에서 우회할 방법이 없다. 웹앱은 최상위 탭(직접 연 https 페이지나
`file://`)에서만 쓸 수 있으니, 현장용으로는 APK를 깔아 두는 쪽이 확실하다.
안드로이드 앱은 화면(HTML/CSS)을 그대로 재사용하고 블루투스만 네이티브로 내렸다.

## 안드로이드 앱 빌드

Gradle이 필요 없다 — `build-tools`만으로 몇 초면 나온다.

```bash
cd android/handset
./build.sh          # out/handset.apk
./build.sh install  # 빌드 + adb 설치
```

필요한 것: Android SDK(`build-tools 36.0.0`, `platforms/android-36`), JDK 17+.
`ANDROID_HOME`이 없으면 `%LOCALAPPDATA%\Android\Sdk`를 본다.

⚠ **서명키(`handset.keystore`)는 저장소에 없다**(`.gitignore`). 처음 빌드할 때 만들어지며,
**키가 바뀌면 기존 설치를 지우고 깔아야 한다**(`adb uninstall kr.tomatopicker.handset`) —
안드로이드가 서명 불일치를 거부한다.

⚠ 앱은 토큰을 `assets/ui.html`의 `DEFAULT_TOKEN`에 기본값으로 담고 있다. 토큰을
바꿨으면 앱의 [접속 토큰] 칸에 새 값을 넣으면 되고, 다시 빌드할 필요는 없다.

## 설치

```bash
scp tools/ble_console.py server@<젯슨>:/home/server/tomato-picker/tools/
scp deploy/ble-console.service server@<젯슨>:/tmp/
ssh server@<젯슨> "sudo install -m 644 /tmp/ble-console.service /etc/systemd/system/ \
  && sudo systemctl daemon-reload && sudo systemctl enable --now ble-console"
```

의존성은 OS 패키지인 `python3-dbus`·`python3-gi`뿐이다(둘 다 이미 있다).
**vision venv를 쓰면 안 된다** — 거기엔 dbus/gi가 없다. 그래서 유닛이 `/usr/bin/python3`을 쓴다.

## 쓰는 법

1. 크롬(안드로이드/윈도우/맥)에서 웹앱을 연다. **아이폰 사파리는 Web Bluetooth를
   지원하지 않는다** — Bluefy 같은 BLE 브라우저가 필요하다.
2. [접속 토큰]에 젯슨의 `/etc/tomato-ble-token` 값을 넣는다(브라우저에 저장된다).
3. [블루투스 연결] → 목록에서 **`tomato-jetson`** 선택.
4. 연결되면 자동으로 인증하고 주소가 카드에 뜬다.

### 명령

| 명령 | 하는 일 |
|---|---|
| `ip` | 주소·대시보드 URL — **주력** |
| `wifi` | 접속 중인 망 + 보이는 망(신호 순) |
| `join <SSID> <비번>` | 그 WiFi로 갈아타기. SSID에 **공백이 있어도 된다** |
| `status` | 서비스·팔(USB 시리얼)·라인 검출 상태 |
| `restart <서비스>` | `tomato-voice`·`line-cam`·`line-follow`·`controller-drive`만 |
| `sys` | 가동시간·온도·디스크·부하 |

## 설계에서 조심한 것들

- **20바이트 청크는 타협이 아니라 안전판이다.** BLE 기본 MTU는 23(페이로드 20)이고
  협상 결과는 상대 스택에 달렸다. 크롬은 보통 517로 올려주지만 그걸 가정하면 안
  올려주는 조합에서 **응답이 잘린 채 조용히 끝난다**. 20이면 어디서든 맞고,
  wifi 스캔(~1KB)도 0.3초면 다 나간다.
- **한글은 청크 경계에서 잘린다**(UTF-8 3바이트). 그래서 응답 끝에 `0x04`를 붙이고
  클라이언트가 **거기까지 모아서 한 번에 디코드**한다. 청크마다 디코드하면 깨진다.
- **`join`은 SSID를 뒤에서 자른다.** 현장 SSID에 공백이 흔하다("Next door 오피스 5G").
  앞에서 자르면 SSID가 토막난다. 마지막 토큰 = 비밀번호, 나머지 전부 = SSID.
- **서비스는 root로 돈다.** `User=server`로 두면 polkit이 `nmcli`를 막는다 —
  세션 없는 데몬에는 권한이 안 붙는다(실측 오류: `Not authorized to control networking`).
  대신 **임의 셸을 열지 않고** 허용 목록의 명령만 실행한다.
- **토큰 인증.** WiFi를 갈아타고 서비스를 재시작하는 서버가 반경 안 누구에게나
  열려 있으면 안 된다. 연결이 끊기면 인증도 풀린다.

## 막힐 때

| 증상 | 볼 곳 |
|---|---|
| 목록에 `tomato-jetson`이 없다 | `systemctl status ble-console` · 로그에 "광고 시작"이 있나 · `busctl --system get-property org.bluez /org/bluez/hci0 org.bluez.LEAdvertisingManager1 ActiveInstances` 가 1인가 |
| 연결은 되는데 응답이 없다 | 토큰이 맞나(`sudo cat /etc/tomato-ble-token`) |
| 브라우저가 "지원하지 않음" | 사파리다. 크롬/엣지로 열 것 |
| `join`이 실패한다 | `wifi`로 SSID 철자·신호 확인. 5GHz 전용 AP인데 젯슨이 못 보는 경우도 있다 |
