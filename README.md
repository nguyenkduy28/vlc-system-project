# Hệ thống truyền thông ánh sáng nhìn thấy (VLC/OWC)

Đây là repository đồ án thiết kế hệ thống truyền thông quang không dây sử dụng ánh sáng nhìn thấy. Hệ thống gồm bo mạch phát/thu, firmware cho STM32F407VET6, chương trình giám sát trên máy tính và tài liệu phục vụ thiết kế, mô phỏng, đo kiểm.

## 1. Tổng quan hệ thống

Hệ thống truyền dữ liệu theo phương pháp điều chế OOK (On-Off Keying):

- Khối phát tạo sóng mang quang bằng PWM, đóng/ngắt sóng mang theo từng bit dữ liệu.
- Khối thu chuyển tín hiệu quang thành tín hiệu điện, tạo dữ liệu bit từ số cạnh thu được trong mỗi chu kỳ bit.
- STM32 thực hiện đóng gói frame, phát dữ liệu, đồng bộ frame thu, kiểm tra checksum và thống kê chất lượng liên kết.
- Máy tính giao tiếp với bo mạch qua UART để gửi lệnh, quan sát frame, điện áp, RTC, lỗi và các chỉ số BER/PER.

Định dạng frame hiện tại:

```text
AA AA AA | D5 | LEN | PAYLOAD | CHECKSUM
```

Trong đó:

- Preamble: 3 byte `0xAA`.
- Sync: `0xD5`.
- LEN: số byte payload, tối đa 32 byte.
- CHECKSUM: `(LEN + tổng các byte PAYLOAD) mod 256`.

Thông số mặc định của firmware:

| Thông số | Giá trị |
|---|---:|
| Vi điều khiển | STM32F407VET6 |
| Tốc độ bit | 100 kbit/s |
| Tần số tick TIM2 | 100 kHz |
| Sóng mang PWM TIM1 | khoảng 1 MHz |
| Ngưỡng nhận bit theo số cạnh | 6 cạnh/bit |
| Điện áp DAC threshold | 1650 mV |
| UART | 115200 baud |

## 2. Cấu trúc repository

```text
vlc-system-project/
├── firmware/   Firmware STM32CubeIDE cho STM32F407VET6
├── gui/        Các chương trình Python giao tiếp và giám sát UART
├── hardware/   Schematic, PCB, thư viện linh kiện và output Altium
├── ref/        Datasheet, tài liệu tham khảo, ảnh đo và kết quả mô phỏng
└── README.md   Tài liệu tổng quan repository
```

### `firmware/`

Project STM32CubeIDE chính nằm trong thư mục này.

- `OWC_Project.ioc`: cấu hình ngoại vi STM32CubeMX.
- `Core/Src/main.c`: khởi tạo ngoại vi và vòng lặp chương trình.
- `Core/Src/app_tasks.c`: điều phối ứng dụng, xử lý UART command và xuất log.
- `Core/Src/app_protocol.c`: định dạng frame và checksum.
- `Core/Src/vlc_tx.c`: phát frame OOK.
- `Core/Src/vlc_rx.c`: lấy mẫu tín hiệu thu và quản lý queue frame/lỗi.
- `Core/Src/rx_sync.c`: máy trạng thái đồng bộ và giải mã frame.
- `Core/Src/bsp_*.c`: UART, ADC monitor, DAC threshold và RTC.
- `Core/Inc/app_config.h`: các thông số cấu hình chính.
- `Drivers/`: thư viện HAL và CMSIS của STMicroelectronics.
- `Debug/`: output sinh ra khi build, không phải mã nguồn chính.

Các tài liệu luồng xử lý nằm tại:

- `firmware/overview.md`
- `firmware/flowtx.md`
- `firmware/flowrx.md`

### `gui/`

Các GUI được viết bằng Python/Tkinter, nhận log từ UART và hiển thị dữ liệu bằng Matplotlib.

Ba chương trình phù hợp với từng chế độ firmware:

| Chế độ firmware | GUI đề xuất |
|---|---|
| `APP_BOARD_ROLE_TX_ONLY` | `owc_tx_only_gui_v5_rtc.py` |
| `APP_BOARD_ROLE_RX_ONLY` | `owc_rx_only_gui_demo_v1.py` hoặc `owc_rx_monitor.py` |
| `APP_BOARD_ROLE_TX_RX_LOOPBACK` | `owc_loopback_gui_v2_tx_rx_tabs.py` |

Các file có hậu tố `v2`, `v3`, `fixed`, `demo` là các phiên bản phát triển được lưu lại để đối chiếu. Khi chạy thử nên chọn file mới nhất phù hợp với `APP_BOARD_ROLE`.

### `hardware/`

Thiết kế phần cứng được thực hiện bằng Altium Designer. Project chính:

```text
hardware/PCB_STM32F4/PCB_STM32F4.PrjPcb
```

Các sheet chính gồm:

- `BLOCK DIAGRAM.SchDoc`: sơ đồ khối hệ thống.
- `MCU INTERFACE.SchDoc`: STM32 và các kết nối ngoại vi.
- `Transmitter_MD.SchDoc`: mạch phát quang.
- `Receiver_VLC.SchDoc`: mạch thu quang.
- `PW SUPPLY.SchDoc`, `PWR_AFE.SchDoc`: nguồn và analog front-end.
- `VOLTAGE_MONITOR.SchDoc`: giám sát các đường nguồn.
- `UART TO USB LOG.SchDoc`: giao tiếp UART–USB.
- `PCB1.PcbDoc`: layout PCB.
- `VLC_SYSTEM.pdf`: bản xuất sơ đồ hệ thống.

Thư mục `History/` và `Project Outputs for PCB_STM32F4/` chứa lịch sử thiết kế và các báo cáo/output do Altium tạo.

### `ref/`

Chứa tài liệu phục vụ lựa chọn linh kiện và kiểm chứng thiết kế:

- Datasheet LED, photodiode, transistor, driver và IC nguồn.
- Tài liệu thiết kế transimpedance amplifier và mạch lọc.
- Tài liệu STM32F407.
- File mô phỏng TINA/Multisim.
- BOM, ảnh oscilloscope, ảnh PCB và sơ đồ trình tự hoạt động.

## 3. Build và nạp firmware

Yêu cầu:

- STM32CubeIDE.
- ST-LINK hoặc bộ nạp tương thích.
- Bo mạch sử dụng STM32F407VET6.

Các bước thực hiện:

1. Mở STM32CubeIDE và import thư mục `firmware/` dưới dạng project hiện có.
2. Chọn chế độ bo mạch trong `firmware/Core/Inc/app_config.h`:

   ```c
   #define APP_BOARD_ROLE APP_BOARD_ROLE_TX_ONLY
   // hoặc APP_BOARD_ROLE_RX_ONLY
   // hoặc APP_BOARD_ROLE_TX_RX_LOOPBACK
   ```

3. Build cấu hình Debug.
4. Kết nối ST-LINK, nạp chương trình và reset bo mạch.
5. Kết nối UART2 với máy tính ở baud rate `115200`.

Không thay đổi code trong `Drivers/` nếu không cần thiết. Các file chứa marker `USER CODE BEGIN/END` là file do STM32CubeMX quản lý; phần tùy chỉnh nên được đặt trong vùng user code để tránh mất khi generate lại.

## 4. Chạy GUI trên máy tính

Yêu cầu Python 3.9 trở lên. Cài các thư viện cần thiết:

```powershell
python -m pip install pyserial matplotlib
```

Ví dụ chạy GUI loopback:

```powershell
python gui/owc_loopback_gui_v2_tx_rx_tabs.py
```

Hoặc chạy GUI theo từng phía:

```powershell
python gui/owc_tx_only_gui_v5_rtc.py
python gui/owc_rx_only_gui_demo_v1.py
```

Sau khi GUI mở:

1. Chọn đúng cổng COM của bo mạch.
2. Đặt baud rate `115200`.
3. Nhấn **Connect**.
4. Kiểm tra log `role` để xác nhận GUI và firmware đang dùng cùng chế độ.

Các lệnh UART được firmware hỗ trợ gồm nhóm điều khiển TX (`tx_payload`, `tx_start`, `tx_stop`, `tx_single`, `tx_carrier_on`, `tx_carrier_off`, `tx_status`) và nhóm RTC (`rtc_get`, `rtc_set`, `rtc_reset`, `rtc_bkp`). Khả năng sử dụng lệnh TX phụ thuộc vào `APP_BOARD_ROLE`.

## 5. Log UART chính

Firmware xuất log định kỳ hoặc theo sự kiện:

| Log | Nội dung |
|---|---|
| `role` | Chế độ hiện tại của firmware |
| `alive_tx`, `alive_rx` | Bộ đếm và trạng thái hoạt động |
| `tx_frame` | Frame đang phát |
| `expected_tx_frame` | Frame tham chiếu ở phía thu |
| `last_rx`, `rx_frame` | Frame nhận gần nhất |
| `link_stats`, `link_quality` | BER, PER và thống kê payload |
| `err_summary`, `err_frame` | Thống kê và chi tiết lỗi |
| `adc_mv` | Điện áp các kênh giám sát |
| `dac_mv` | Điện áp ngưỡng comparator |
| `rtc` | Thời gian và trạng thái RTC/LSE/VBAT |

## 6. Kiểm thử cơ bản

### TX-only

- Kiểm tra PWM carrier tại đầu ra TIM1.
- Quan sát `tx_frame` và bộ đếm `tx_frames`.
- Thử `tx_start`, `tx_stop`, `tx_single` và thay payload.
- Đối chiếu tốc độ bit và dạng sóng OOK trên oscilloscope.

### RX-only

- Cấp tín hiệu quang từ bộ phát vào mạch thu.
- Theo dõi `last_edge_delta`, `last_raw_bit` và trạng thái FSM.
- Kiểm tra frame nhận, checksum, frame error và BER/PER.
- Điều chỉnh ngưỡng DAC khi chất lượng tín hiệu thay đổi.

### Loopback

- Build firmware với `APP_BOARD_ROLE_TX_RX_LOOPBACK`.
- So sánh trực tiếp frame TX và RX trong GUI loopback.
- Theo dõi lỗi theo vị trí bit, chất lượng liên kết và các đường nguồn.

## 7. Ghi chú

- Giá trị điện áp, ngưỡng cạnh và DAC cần được hiệu chỉnh theo phiên bản phần cứng và kết quả đo thực tế.
- Không commit thêm output build hoặc file tạm của IDE vào phần mã nguồn phát hành.
- Datasheet và tài liệu trong `ref/` thuộc về các nhà sản xuất hoặc tác giả tương ứng và chỉ được lưu để tham khảo kỹ thuật.
