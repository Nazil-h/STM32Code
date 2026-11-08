# STM32Code - Robot Driver & Camera Servo Firmware

Firmware berbasis **STM32 HAL (STM32F401RCT6)** untuk mengendalikan penggerak motor DC ganda (*differential drive*) dan motor servo kamera melalui komunikasi serial UART. Repositori ini merupakan bagian *embedded controller* yang terintegrasi dengan ekosistem aplikasi mobile pengendali robot.

## 🔗 Ekosistem Terkait
* **[RobotHub](https://github.com/Nazil-h/RobotHub)** — Aplikasi kamera yang terhubung ke mikrokontroler dan menjembatani aliran video serta instruksi kontrol.
* **[App_RobotDriver](https://github.com/Nazil-h/App_RobotDriver)** — Aplikasi *remote controller* untuk mengirimkan perintah navigasi dan sudut kamera secara *real-time*.

---

## ✨ Fitur Utama
* **Kontrol Motor DC Ganda (PWM & Arah):** Pengaturan kecepatan dan arah putaran motor kiri dan kanan secara independen menggunakan `TIM2`.
* **Kontrol Servo Kamera:** Pengaturan sudut kemiringan (*tilt/pan*) kamera menggunakan sinyal PWM pada `TIM3`.
* **Protokol Komunikasi Ganda (USART1):**
  * **Format CSV:** Kendali presisi untuk kecepatan motor kiri, motor kanan, dan sudut servo (`kiri,kanan,servo\n`).
  * **Perintah Karakter Tunggal:** Kendali cepat arah dasar (`F`, `B`, `L`, `R`, `S`).
* **Auto-Failsafe Timeout:** Menghentikan seluruh pergerakan motor secara otomatis apabila tidak ada data perintah yang diterima selama lebih dari **1000 ms (1 detik)** untuk mencegah robot bergerak liar saat koneksi terputus.

---

## 🛠️ Spesifikasi Perangkat Keras & Pinout

Target Board: **STM32F401RCT6**

| Fungsi | Pin STM32 | Mode / Peripheral | Keterangan |
| :--- | :--- | :--- | :--- |
| **PWM Motor Kiri** | `PA0` | `TIM2_CH1` | Pin Enable A (ENA) pada Driver Motor |
| **PWM Motor Kanan** | `PA1` | `TIM2_CH2` | Pin Enable B (ENB) pada Driver Motor |
| **Arah Motor Kiri** | `PA2`, `PA3` | `GPIO_Output` | Pin Input 1 & 2 (IN1, IN2) Driver Motor |
| **Arah Motor Kanan** | `PB0`, `PB1` | `GPIO_Output` | Pin Input 3 & 4 (IN3, IN4) Driver Motor |
| **PWM Servo Kamera** | `PA6` | `TIM3_CH1` | Sinyal kontrol sudut motor servo |
| **UART TX** | `PA9` | `USART1_TX` | Jalur transmisi serial (115200 bps) |
| **UART RX** | `PA10` | `USART1_RX` | Jalur penerimaan serial berbasis *Interrupt* |

---

## 📡 Protokol Komunikasi Serial

Komunikasi menggunakan **USART1** dengan konfigurasi **Baud Rate 115200, 8-N-1** dan diakhiri karakter *newline* (`\n`).

### 1. Format CSV (Kontrol Presisi)
Format paket data:
```text
<kecepatan_kiri>,<kecepatan_kanan>,<sudut_servo>\n
```
* **`kecepatan_kiri` & `kecepatan_kanan`**: Nilai positif untuk maju, nilai negatif untuk mundur, dan `0` untuk berhenti.
* **`sudut_servo`**: Sudut target servo kamera (`0` hingga `180` derajat).
* **Contoh:** `200,200,90\n` (Maju lurus dengan kecepatan 200, posisi kamera 90 derajat).

### 2. Perintah Karakter Tunggal (Kontrol Cepat)
| Karakter | Aksi |
| :---: | :--- |
| `F` | Maju (*Forward*) |
| `B` | Mundur (*Backward*) |
| `L` | Belok Kiri (*Left*) |
| `R` | Belok Kanan (*Right*) |
| `S` | Berhenti (*Stop*) |

---

## 🚀 Cara Instalasi & Penggunaan

1. **Clone Repositori**
   ```bash
   git clone https://github.com/Nazil-h/STM32Code.git
   ```
2. **Buka Proyek di STM32CubeIDE**
   * Pilih **File -> Open Projects from File System...** lalu arahkan ke direktori hasil *clone*.
3. **Konfigurasi Peripheral (`.ioc`)**
   * Pastikan konfigurasi *clock* dan prescaler pada `TIM2` (PWM Motor), `TIM3` (50Hz untuk Servo), serta `USART1` (*Global Interrupt* aktif) sesuai dengan kebutuhan perangkat keras Anda.
4. **Build & Flash**
   * Hubungkan board **STM32F401RCT6** menggunakan ST-Link.
   * Klik **Run / Debug** di STM32CubeIDE untuk mengunggah *firmware* ke mikrokontroler.

---

## 📄 Lisensi
Proyek ini dikembangkan untuk keperluan riset dan pengembangan sistem kendali robotika. Silakan gunakan dan modifikasi sesuai kebutuhan.