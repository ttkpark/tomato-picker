package kr.tomatopicker.handset;

import android.Manifest;
import android.annotation.SuppressLint;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattDescriptor;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothManager;
import android.bluetooth.le.BluetoothLeScanner;
import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanFilter;
import android.bluetooth.le.ScanResult;
import android.bluetooth.le.ScanSettings;
import android.content.Context;
import android.content.pm.PackageManager;
import android.os.Build;
import android.os.Handler;
import android.os.Looper;
import android.os.ParcelUuid;

import java.io.ByteArrayOutputStream;
import java.nio.charset.StandardCharsets;
import java.util.ArrayDeque;
import java.util.Collections;
import java.util.UUID;

/**
 * 젯슨의 BLE 콘솔(Nordic UART Service)에 붙어 줄 단위 텍스트를 주고받는다.
 *
 * <p>왜 네이티브인가 — 크롬의 Web Bluetooth는 아티팩트처럼 <b>샌드박스 iframe</b>
 * 안에서는 권한정책에 막혀 기기 목록조차 못 띄운다. 화면은 WebView로 그대로 쓰되
 * 블루투스만 여기로 내렸다.
 *
 * <p>⚠ <b>안드로이드 GATT는 한 번에 한 작업만 받는다.</b> 쓰기를 연달아 던지면
 * 조용히 버려진다(반환값 false조차 안 오는 경우가 있다). 그래서 20바이트 조각을
 * 큐에 넣고 onCharacteristicWrite 콜백이 올 때마다 다음 것을 보낸다. 알림 활성화
 * (CCCD 쓰기)도 같은 큐를 타야 해서, 그게 끝난 뒤에야 준비 완료로 본다.
 *
 * <p>⚠ 응답은 <b>0x04가 올 때까지 모았다가</b> 한 번에 UTF-8 디코드한다. 한글은
 * 3바이트라 20바이트 경계에서 글자가 잘리는데, 조각마다 디코드하면 깨진다.
 */
public class BleLink {

    public static final UUID NUS_SVC = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID NUS_RX  = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e");
    public static final UUID NUS_TX  = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e");
    private static final UUID CCCD   = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");

    private static final int EOT = 0x04;
    private static final int WRITE_CHUNK = 20;
    private static final long SCAN_TIMEOUT_MS = 15000;

    /** 화면으로 올려보내는 사건들. 전부 메인 스레드에서 부른다. */
    public interface Listener {
        /** state: "off" | "wait" | "live" */
        void onState(String state, String label, String note);
        void onText(String text);
    }

    private final Context ctx;
    private final Listener listener;
    private final Handler main = new Handler(Looper.getMainLooper());

    private BluetoothAdapter adapter;
    private BluetoothLeScanner scanner;
    private BluetoothGatt gatt;
    private BluetoothGattCharacteristic rxChar, txChar;

    private final ArrayDeque<byte[]> writeQueue = new ArrayDeque<>();
    private boolean writing = false;
    private final ByteArrayOutputStream rxBuf = new ByteArrayOutputStream();
    private boolean scanning = false;

    public BleLink(Context ctx, Listener listener) {
        this.ctx = ctx.getApplicationContext();
        this.listener = listener;
        BluetoothManager bm = (BluetoothManager) this.ctx.getSystemService(Context.BLUETOOTH_SERVICE);
        if (bm != null) adapter = bm.getAdapter();
    }

    // ------------------------------------------------------------------
    // 권한
    // ------------------------------------------------------------------

    public static String[] neededPermissions() {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            return new String[]{Manifest.permission.BLUETOOTH_SCAN,
                                Manifest.permission.BLUETOOTH_CONNECT};
        }
        return new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
    }

    public boolean hasPermissions() {
        for (String p : neededPermissions()) {
            if (ctx.checkSelfPermission(p) != PackageManager.PERMISSION_GRANTED) return false;
        }
        return true;
    }

    public boolean isConnected() {
        return gatt != null && rxChar != null && txChar != null;
    }

    // ------------------------------------------------------------------
    // 스캔 → 연결
    // ------------------------------------------------------------------

    @SuppressLint("MissingPermission")
    public void connect() {
        if (adapter == null) { fail("이 기기에 블루투스가 없습니다"); return; }
        if (!adapter.isEnabled()) { fail("블루투스가 꺼져 있습니다 — 켜고 다시 누르세요"); return; }
        if (!hasPermissions()) { fail("블루투스 권한이 필요합니다 — 권한을 허용한 뒤 다시 누르세요"); return; }
        if (isConnected()) return;

        scanner = adapter.getBluetoothLeScanner();
        if (scanner == null) { fail("BLE 스캐너를 열 수 없습니다"); return; }

        state("wait", "찾는 중", "tomato-jetson을 찾고 있습니다…");
        ScanFilter filter = new ScanFilter.Builder()
                .setServiceUuid(new ParcelUuid(NUS_SVC)).build();
        ScanSettings settings = new ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build();
        scanning = true;
        scanner.startScan(Collections.singletonList(filter), settings, scanCallback);
        main.postDelayed(scanTimeout, SCAN_TIMEOUT_MS);
    }

    private final Runnable scanTimeout = new Runnable() {
        @Override public void run() {
            if (!scanning) return;
            stopScan();
            fail("15초 동안 찾지 못했습니다 — 젯슨 가까이(10m 안)에서 다시 시도하세요.\n"
                    + "젯슨에서: systemctl status ble-console");
        }
    };

    @SuppressLint("MissingPermission")
    private void stopScan() {
        if (scanning && scanner != null) {
            scanning = false;
            try { scanner.stopScan(scanCallback); } catch (Exception ignored) { }
        }
        main.removeCallbacks(scanTimeout);
    }

    private final ScanCallback scanCallback = new ScanCallback() {
        @SuppressLint("MissingPermission")
        @Override public void onScanResult(int callbackType, ScanResult result) {
            if (!scanning) return;
            stopScan();
            BluetoothDevice dev = result.getDevice();
            String name = dev.getName();
            state("wait", "연결 중", "찾았습니다: " + (name != null ? name : dev.getAddress()));
            gatt = dev.connectGatt(ctx, false, gattCallback, BluetoothDevice.TRANSPORT_LE);
        }

        @Override public void onScanFailed(int errorCode) {
            stopScan();
            fail("스캔 실패 (코드 " + errorCode + ")");
        }
    };

    // ------------------------------------------------------------------
    // GATT
    // ------------------------------------------------------------------

    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {

        @SuppressLint("MissingPermission")
        @Override public void onConnectionStateChange(BluetoothGatt g, int status, int newState) {
            if (newState == BluetoothGatt.STATE_CONNECTED) {
                state("wait", "탐색 중", "서비스를 찾는 중…");
                g.discoverServices();
            } else if (newState == BluetoothGatt.STATE_DISCONNECTED) {
                cleanup();
                state("off", "끊김", "연결이 끊겼습니다. 다시 연결하면 자동으로 재인증합니다.");
            }
        }

        @SuppressLint("MissingPermission")
        @Override public void onServicesDiscovered(BluetoothGatt g, int status) {
            BluetoothGattService svc = g.getService(NUS_SVC);
            if (svc == null) { fail("젯슨에서 NUS 서비스를 찾지 못했습니다"); return; }
            rxChar = svc.getCharacteristic(NUS_RX);
            txChar = svc.getCharacteristic(NUS_TX);
            if (rxChar == null || txChar == null) { fail("NUS 캐릭터리스틱이 없습니다"); return; }

            // 알림 켜기 — 로컬 등록 + CCCD 쓰기, 둘 다 해야 실제로 온다.
            g.setCharacteristicNotification(txChar, true);
            BluetoothGattDescriptor d = txChar.getDescriptor(CCCD);
            if (d == null) { fail("CCCD 디스크립터가 없습니다"); return; }
            byte[] on = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE;
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                g.writeDescriptor(d, on);
            } else {
                d.setValue(on);
                g.writeDescriptor(d);
            }
        }

        @Override public void onDescriptorWrite(BluetoothGatt g, BluetoothGattDescriptor d, int status) {
            if (!CCCD.equals(d.getUuid())) return;
            if (status != BluetoothGatt.GATT_SUCCESS) { fail("알림 활성화 실패 (status " + status + ")"); return; }
            state("live", "연결됨", null);
        }

        // API 33+ 는 이 쪽으로 온다.
        @Override public void onCharacteristicChanged(BluetoothGatt g,
                BluetoothGattCharacteristic c, byte[] value) {
            if (NUS_TX.equals(c.getUuid())) feed(value);
        }

        @SuppressWarnings("deprecation")
        @Override public void onCharacteristicChanged(BluetoothGatt g,
                BluetoothGattCharacteristic c) {
            if (NUS_TX.equals(c.getUuid())) feed(c.getValue());
        }

        @Override public void onCharacteristicWrite(BluetoothGatt g,
                BluetoothGattCharacteristic c, int status) {
            writing = false;
            pump();
        }
    };

    /** 들어온 바이트를 모으다가 0x04를 만나면 한 덩어리로 올려보낸다. */
    private void feed(byte[] value) {
        if (value == null) return;
        for (byte b : value) {
            if ((b & 0xFF) == EOT) {
                final String text = new String(rxBuf.toByteArray(), StandardCharsets.UTF_8);
                rxBuf.reset();
                if (!text.trim().isEmpty()) main.post(() -> listener.onText(text));
            } else {
                rxBuf.write(b);
            }
        }
    }

    // ------------------------------------------------------------------
    // 송신
    // ------------------------------------------------------------------

    public void send(String line) {
        if (!isConnected()) { note("연결되어 있지 않습니다"); return; }
        byte[] data = (line + "\n").getBytes(StandardCharsets.UTF_8);
        synchronized (writeQueue) {
            for (int i = 0; i < data.length; i += WRITE_CHUNK) {
                int end = Math.min(i + WRITE_CHUNK, data.length);
                byte[] chunk = new byte[end - i];
                System.arraycopy(data, i, chunk, 0, end - i);
                writeQueue.add(chunk);
            }
        }
        pump();
    }

    @SuppressLint("MissingPermission")
    private void pump() {
        byte[] chunk;
        synchronized (writeQueue) {
            if (writing || writeQueue.isEmpty() || gatt == null || rxChar == null) return;
            chunk = writeQueue.poll();
            writing = true;
        }
        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                writeModern(chunk);
            } else {
                writeLegacy(chunk);
            }
        } catch (Exception e) {
            writing = false;
            note("전송 실패: " + e.getMessage());
        }
    }

    @SuppressWarnings("deprecation")
    @SuppressLint("MissingPermission")
    private void writeLegacy(byte[] chunk) {
        rxChar.setValue(chunk);
        rxChar.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
        gatt.writeCharacteristic(rxChar);
    }

    // API 33+ 의 writeCharacteristic(char, value, type) 오버로드를 쓰기 위한 래퍼.
    @SuppressLint("MissingPermission")
    private void writeModern(byte[] chunk) {
        gatt.writeCharacteristic(rxChar, chunk, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);
    }

    // ------------------------------------------------------------------

    @SuppressLint("MissingPermission")
    public void disconnect() {
        stopScan();
        if (gatt != null) {
            try { gatt.disconnect(); gatt.close(); } catch (Exception ignored) { }
        }
        cleanup();
        state("off", "끊김", null);
    }

    private void cleanup() {
        rxChar = null; txChar = null;
        synchronized (writeQueue) { writeQueue.clear(); writing = false; }
        rxBuf.reset();
        if (gatt != null) { try { gatt.close(); } catch (Exception ignored) { } gatt = null; }
    }

    private void fail(String msg) {
        cleanup();
        state("off", "끊김", msg);
    }

    private void note(String msg) {
        main.post(() -> listener.onText(msg));
    }

    private void state(String s, String label, String note) {
        main.post(() -> listener.onState(s, label, note));
    }
}
