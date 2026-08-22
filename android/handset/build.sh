#!/usr/bin/env bash
# 젯슨 핸드셋 APK 빌드 — Gradle 없이 build-tools만으로.
#
# 왜 Gradle을 안 쓰나: 이 앱은 액티비티 하나 + 클래스 둘이다. Gradle을 붙이면
# 래퍼가 수백 MB를 받고 AGP/Kotlin 버전 궁합을 맞춰야 하는데, 얻는 게 없다.
# aapt2 → javac → d8 → zipalign → apksigner 다섯 단계면 끝나고, 몇 초 걸린다.
#
# 쓰기:  ./build.sh          빌드만
#        ./build.sh install  빌드 + adb 설치
set -euo pipefail

SDK="${ANDROID_HOME:-$LOCALAPPDATA/Android/Sdk}"
SDK="${SDK//\\//}"                       # 윈도우 역슬래시 정리
BTV="${BUILD_TOOLS:-36.0.0}"
API="${TARGET_API:-36}"
MIN_API="${MIN_API:-26}"

BT="$SDK/build-tools/$BTV"
PLAT="$SDK/platforms/android-$API/android.jar"
ADB="$SDK/platform-tools/adb"
JB="/c/Program Files/Android/Android Studio/jbr/bin"
JAVAC="${JAVAC:-$JB/javac}"
KEYTOOL="${KEYTOOL:-$JB/keytool}"

cd "$(dirname "$0")"
[ -f "$PLAT" ] || { echo "android.jar 없음: $PLAT"; exit 1; }

# 윈도우 실행파일은 .exe/.bat이 붙는다. 리눅스/맥이면 그대로.
x() { if [ -f "$1.exe" ]; then echo "$1.exe"; elif [ -f "$1.bat" ]; then echo "$1.bat"; else echo "$1"; fi; }
AAPT2="$(x "$BT/aapt2")"; D8="$(x "$BT/d8")"
ZIPALIGN="$(x "$BT/zipalign")"; APKSIGNER="$(x "$BT/apksigner")"

# cygpath가 있으면(Git Bash) 자바에 넘길 경로를 윈도우식으로 바꾼다.
w() { if command -v cygpath >/dev/null 2>&1; then cygpath -w "$1"; else echo "$1"; fi; }

rm -rf out && mkdir -p out/gen out/classes

echo "[1/6] 리소스 컴파일"
"$AAPT2" compile --dir res -o out/res.zip

echo "[2/6] 링크 (+assets)"
"$AAPT2" link -o out/base.apk -I "$PLAT" --manifest AndroidManifest.xml \
  -R out/res.zip --java out/gen -A assets \
  --min-sdk-version "$MIN_API" --target-sdk-version "$API" --auto-add-overlay

echo "[3/6] javac"
# --release 11: d8이 먹는 바이트코드. android.jar는 classpath로만 준다.
"$JAVAC" -encoding UTF-8 --release 11 -nowarn -cp "$(w "$PLAT")" -d out/classes \
  $(find java out/gen -name '*.java')

echo "[4/6] d8 (dex)"
"$D8" --lib "$(w "$PLAT")" --min-api "$MIN_API" --output out \
  $(find out/classes -name '*.class')

echo "[5/6] 패키징 + 정렬"
cp out/base.apk out/unsigned.apk
python -c "
import zipfile
z = zipfile.ZipFile('out/unsigned.apk', 'a', zipfile.ZIP_DEFLATED)
z.write('out/classes.dex', 'classes.dex')
z.close()
"
"$ZIPALIGN" -f -p 4 out/unsigned.apk out/aligned.apk

echo "[6/6] 서명"
# ⚠ 디버그용 자체 서명키다. 스토어에 올릴 물건이 아니라 현장용 사이드로드다.
#   키가 바뀌면 기존 설치를 지우고 깔아야 한다(서명 불일치).
if [ ! -f handset.keystore ]; then
  "$KEYTOOL" -genkeypair -keystore handset.keystore -storepass android -keypass android \
    -alias handset -dname "CN=Tomato Handset,O=tomato-picker,C=KR" \
    -keyalg RSA -keysize 2048 -validity 10000 >/dev/null 2>&1
  echo "  새 서명키 생성: handset.keystore"
fi
"$APKSIGNER" sign --ks handset.keystore --ks-pass pass:android --key-pass pass:android \
  --ks-key-alias handset --out out/handset.apk out/aligned.apk 2>/dev/null

echo
echo "완성: $(pwd)/out/handset.apk  ($(du -h out/handset.apk | cut -f1))"

if [ "${1:-}" = "install" ]; then
  echo "설치 중…"
  "$(x "$ADB")" install -r out/handset.apk
fi
