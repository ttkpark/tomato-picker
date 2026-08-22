package kr.tomatopicker.handset;

import android.app.Activity;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.res.Configuration;
import android.net.Uri;
import android.os.Bundle;
import android.webkit.JavascriptInterface;
import android.webkit.WebResourceRequest;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.Toast;

import org.json.JSONObject;

/**
 * 화면은 WebView(assets/ui.html), 블루투스는 네이티브({@link BleLink}).
 *
 * <p>이 구조를 고른 이유 — 같은 UI를 크롬 웹앱으로도 만들어 봤는데, 아티팩트처럼
 * <b>샌드박스 iframe</b>에서 열리면 Web Bluetooth가 권한정책에 막혀 기기 목록조차
 * 안 뜬다. 화면은 그대로 재사용하고 블루투스만 네이티브로 내리면 그 문제가 사라지고,
 * 앱을 한 번 깔아두면 브라우저·네트워크와 무관하게 늘 열린다.
 */
public class MainActivity extends Activity implements BleLink.Listener {

    private static final int REQ_PERM = 1;
    private static final String PREFS = "handset";
    private static final String KEY_TOKEN = "token";

    private WebView web;
    private BleLink ble;
    private boolean pageReady = false;

    @Override
    protected void onCreate(Bundle saved) {
        super.onCreate(saved);

        ble = new BleLink(this, this);

        web = new WebView(this);
        web.getSettings().setJavaScriptEnabled(true);
        web.getSettings().setDomStorageEnabled(true);
        web.setWebViewClient(new WebViewClient() {
            @Override
            public boolean shouldOverrideUrlLoading(WebView v, WebResourceRequest req) {
                // 대시보드 링크는 앱 안이 아니라 브라우저로 — 앱이 웹뷰에 갇히면 안 된다.
                startActivity(new Intent(Intent.ACTION_VIEW, req.getUrl()));
                return true;
            }

            @Override
            public void onPageFinished(WebView v, String url) {
                pageReady = true;
                // 시스템 다크모드를 페이지에 그대로 물려준다.
                boolean dark = (getResources().getConfiguration().uiMode
                        & Configuration.UI_MODE_NIGHT_MASK) == Configuration.UI_MODE_NIGHT_YES;
                js("window.applyTheme(" + (dark ? "'dark'" : "'light'") + ")");
                js("window.applyToken(" + q(loadToken()) + ")");
                if (!ble.hasPermissions()) {
                    requestPermissions(BleLink.neededPermissions(), REQ_PERM);
                }
            }
        });
        setContentView(web);
        web.loadUrl("file:///android_asset/ui.html");
        web.addJavascriptInterface(new Bridge(), "Native");
    }

    // ------------------------------------------------------------------
    // JS ← → 네이티브
    // ------------------------------------------------------------------

    private class Bridge {
        @JavascriptInterface
        public void connect() {
            runOnUiThread(() -> {
                if (!ble.hasPermissions()) {
                    requestPermissions(BleLink.neededPermissions(), REQ_PERM);
                    return;
                }
                ble.connect();
            });
        }

        @JavascriptInterface
        public void disconnect() {
            runOnUiThread(() -> ble.disconnect());
        }

        @JavascriptInterface
        public void send(String line) {
            ble.send(line);
        }

        @JavascriptInterface
        public void saveToken(String token) {
            getSharedPreferences(PREFS, Context.MODE_PRIVATE)
                    .edit().putString(KEY_TOKEN, token).apply();
        }

        @JavascriptInterface
        public void copy(String text) {
            runOnUiThread(() -> {
                android.content.ClipboardManager cm =
                        (android.content.ClipboardManager) getSystemService(Context.CLIPBOARD_SERVICE);
                if (cm != null) {
                    cm.setPrimaryClip(android.content.ClipData.newPlainText("주소", text));
                    Toast.makeText(MainActivity.this, "복사됨: " + text, Toast.LENGTH_SHORT).show();
                }
            });
        }

        @JavascriptInterface
        public void openUrl(String url) {
            runOnUiThread(() -> {
                try {
                    startActivity(new Intent(Intent.ACTION_VIEW, Uri.parse(url)));
                } catch (Exception e) {
                    Toast.makeText(MainActivity.this, "열 수 없습니다: " + url,
                            Toast.LENGTH_SHORT).show();
                }
            });
        }
    }

    private String loadToken() {
        SharedPreferences p = getSharedPreferences(PREFS, Context.MODE_PRIVATE);
        String t = p.getString(KEY_TOKEN, "");
        return t == null ? "" : t;
    }

    private void js(String code) {
        runOnUiThread(() -> web.evaluateJavascript(code, null));
    }

    /** 자바 문자열 → JS 리터럴. 따옴표·줄바꿈·유니코드를 안전하게 넘긴다. */
    private static String q(String s) {
        return JSONObject.quote(s == null ? "" : s);
    }

    // ------------------------------------------------------------------
    // BleLink.Listener
    // ------------------------------------------------------------------

    @Override
    public void onState(String state, String label, String note) {
        if (!pageReady) return;
        js("window.onNativeState(" + q(state) + "," + q(label) + "," + q(note) + ")");
    }

    @Override
    public void onText(String text) {
        if (!pageReady) return;
        js("window.onNativeText(" + q(text) + ")");
    }

    @Override
    public void onRequestPermissionsResult(int req, String[] perms, int[] results) {
        super.onRequestPermissionsResult(req, perms, results);
        if (req != REQ_PERM) return;
        boolean ok = results.length > 0;
        for (int r : results) if (r != android.content.pm.PackageManager.PERMISSION_GRANTED) ok = false;
        if (ok) {
            onText("블루투스 권한을 받았습니다. [블루투스 연결]을 누르세요.");
        } else {
            onText("블루투스 권한이 거부됐습니다 — 설정 > 앱 > 젯슨 핸드셋 > 권한에서 "
                    + "'근처 기기'를 허용해야 스캔할 수 있습니다.");
        }
    }

    @Override
    protected void onDestroy() {
        if (ble != null) ble.disconnect();
        super.onDestroy();
    }
}
