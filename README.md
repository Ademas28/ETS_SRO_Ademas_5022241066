# ETS SRO - 3D Mapping dengan Drone (CoppeliaSim)

Ademas Fazri Sunaryo - NRP 5022241066
Departemen Teknik Elektro, Institut Teknologi Sepuluh Nopember (ITS)

## Deskripsi

Simulasi 3D mapping menggunakan drone quadcopter di CoppeliaSim yang dikendalikan lewat Python (ZMQ Remote API). Drone terbang menyapu arena 4 x 4 m dengan pola lawnmower, memindai lingkungan dengan sensor 3D laser, lalu hasilnya dijadikan peta 3D (point cloud) yang diwarnai berdasarkan ketinggian.

Fitur utama:
- Perencanaan jalur lawnmower dengan ketinggian adaptif terhadap rintangan
- Transformasi titik sensor ke frame global (sensor -> base -> world)
- Voxel downsampling (0,05 m) untuk mengurangi jumlah titik
- Color mapping berdasarkan ketinggian (colormap RdYlGn_r) dan plot 3D beserta jalur drone

## Isi Repository

| File | Keterangan |
|------|------------|
| `drone3d.py` | Program utama (kontrol drone, scan, pemrosesan point cloud, plot) |
| `droneee.ttt` | Scene CoppeliaSim (drone, arena, sofa, ConcreteBlock) |
| `Laporan3DMap_Ademas.pdf` | Laporan lengkap |

## Cara Menjalankan

1. Install CoppeliaSim, lalu buka `droneee.ttt`.
2. Install library Python:
   ```
   pip install coppeliasim-zmqremoteapi-client numpy matplotlib
   ```
3. Jalankan program (CoppeliaSim tetap terbuka):
   ```
   python drone3d.py
   ```
4. Setelah drone selesai terbang, peta 3D akan ditampilkan.

## Parameter Utama

| Parameter | Nilai |
|-----------|-------|
| Area scan | [-2, 2] x [-2, 2] m |
| Ketinggian terbang dasar | 1,5 m |
| Jarak antar jalur | 1,0 m |
| Kecepatan target | 0,4 m/s |
| Ukuran voxel | 0,05 m |

## Hasil

Peta 3D menampilkan lantai, sofa (puncak sekitar 1 m), dan ConcreteBlock dengan manekin (puncak sekitar 2 m). Warna hijau menunjukkan area rendah dan merah menunjukkan area tinggi. Garis biru adalah jalur terbang drone. Penjelasan dan gambar lengkap ada di laporan PDF.
