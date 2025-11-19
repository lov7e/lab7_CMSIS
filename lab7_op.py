from bluepy.btle import Peripheral, Scanner, DefaultDelegate, UUID, BTLEException, BTLEDisconnectError
import time, sys, signal

# ---------- 掃描階段 ----------
class ScanDelegate(DefaultDelegate):
    def handleDiscovery(self, dev, isNewDev, isNewData):
        if isNewDev:
            print("Discovered device", dev.addr)
        elif isNewData:
            print("Received new data from", dev.addr)

def safe_scan(timeout=6.0):
    """安全掃描：確保 Ctrl+C 也能正常結束"""
    scanner = Scanner().withDelegate(ScanDelegate())
    devices = []
    try:
        print(" Scanning for BLE devices...")
        devices = scanner.scan(timeout)
    except KeyboardInterrupt:
        print("\n Scan interrupted by user.")
    except BTLEDisconnectError as e:
        print(f" Scan error: {e}")
    finally:
        try:
            scanner.stop()
            print(" Scanner stopped cleanly.")
        except Exception:
            pass
        time.sleep(0.5)
    return devices

devices = safe_scan(6.0)

if not devices:
    print("No devices found, exiting.")
    sys.exit(0)

addr_list = []
for i, dev in enumerate(devices):
    print(f"{i}: {dev.addr} (RSSI={dev.rssi} dB)")
    addr_list.append(dev.addr)
    for (adtype, desc, value) in dev.getScanData():
        print(f"  {desc} = {value}")

number = input("Enter device number to connect: ")
num = int(number)
target_addr = addr_list[num]

# ---------- 通知處理 delegate ----------
class NotifyDelegate(DefaultDelegate):
    def handleNotification(self, cHandle, data):
        print(f" Notification from handle {cHandle}: {data.hex()}")

# ---------- 連線與訂閱 ----------
def connect_device(addr):
    """嘗試以 random/public 兩種模式連線"""
    for addrType in ["random", "public"]:
        try:
            print(f" Trying to connect ({addrType})...")
            dev = Peripheral(addr, addrType=addrType)
            print(f" Connected with addrType={addrType}")
            return dev
        except Exception as e:
            print(f" Failed with {addrType}: {e}")
    raise Exception(" Unable to connect with either address type")

dev = None
try:
    dev = connect_device(target_addr)
    dev.setDelegate(NotifyDelegate())
    time.sleep(1.5)  # 等待 stack 初始化

    CHARACTERISTIC_UUID = UUID("aabbccdd-eeff-1122-3344-556600000002")
    chars = dev.getCharacteristics(uuid=CHARACTERISTIC_UUID)
    if not chars:
        raise Exception(" Characteristic not found.")
    ch = chars[0]
    handle = ch.getHandle()
    print(f"Found characteristic: {ch}, handle = {handle}")

    # 找 CCCD descriptor
    descs = dev.getDescriptors(handle - 1, handle + 5)
    cccd_handle = None
    for d in descs:
        print(f"Descriptor {d.uuid} at handle {d.handle}")
        if str(d.uuid) == "00002902-0000-1000-8000-00805f9b34fb":
            cccd_handle = d.handle

    if cccd_handle is None:
        cccd_handle = handle + 1
        print(f" CCCD descriptor not found, fallback to handle {cccd_handle}")
    else:
        print(f" Found CCCD descriptor at handle {cccd_handle}")

    # 啟用通知或指示
    enabled = False
    for mode, value in [("notification", b"\x01\x00"), ("indication", b"\x02\x00")]:
        try:
            dev.writeCharacteristic(cccd_handle, value, withResponse=True)
            print(f" {mode.capitalize()} enabled successfully!")
            enabled = True
            break
        except BTLEException as e:
            print(f" {mode} enable failed: {e}")

    if not enabled:
        print(" Failed to enable notification/indication.")
    else:
        print(" Waiting for notifications (press Ctrl+C to stop)...")
        while True:
            if dev.waitForNotifications(5.0):
                continue
            print(" No notification received in 5 seconds.")

except KeyboardInterrupt:
    print("\n Interrupted by user.")
except Exception as e:
    print(f" Error: {e}")
finally:
    # 關閉訂閱與連線
    if dev:
        try:
            dev.writeCharacteristic(cccd_handle, b"\x00\x00", withResponse=True)
            print(" Unsubscribed.")
        except Exception:
            pass
        try:
            dev.disconnect()
            print(" Disconnected.")
        except Exception:
            pass
