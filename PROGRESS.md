# Proje İlerleme Notları

Son güncelleme: 17 Ağustos 2026

## Mevcut durum

Gazebo tabanlı şerit takip simülasyonu düz yolun yanı sıra kapalı dairesel
pistte de çalışır durumda. Kapalı pist değişiklikleri henüz commit edilmedi.

- Son merge commit'i: `ff55dc7`
- Pull request: `#3 - Add Gazebo lane-following simulation`
- Aktif branch: `main`
- Son doğrulama: 6 test, 0 hata, 0 başarısızlık, 2 atlanan telif testi

## Şu ana kadar yapılanlar

- Video dosyasından çalışan şerit algılama düğümü, Gazebo kamera konusundan da
  görüntü alabilecek şekilde geliştirildi.
- `cv_bridge` ve sensor-data QoS profili ile
  `/lane_robot/front_camera/image_raw` konusu bağlandı.
- Kamera ve video girişleri için ayrı görüntü bölgesi, çizgi eğimi ve referans
  noktası ayarları eklendi.
- Şerit merkezinden piksel sapmasını `lane_offset` konusu üzerinden yayımlayan
  algılama hattı oluşturuldu.
- Sapmayı `LEFT`, `RIGHT` ve `STRAIGHT` kararlarına dönüştüren yön düğümünün
  gereksiz tekrar logları azaltıldı.
- `lane_controller` düğümü eklendi. Bu düğüm:
  - sapmaya orantılı açısal hız üretir,
  - `/lane_robot/cmd_vel` konusuna hız komutu gönderir,
  - şerit verisi 0.5 saniyeden uzun süre kesilirse robotu güvenli biçimde
    durdurur.
- Diferansiyel sürüşlü robot, ön kamera, şeritli yol ve Gazebo launch dosyasını
  içeren `lane_simulation_pkg` oluşturuldu.
- DDS mesaj ayrıştırma hatalarını önlemek için simülasyon launch dosyasına
  `ROS_DOMAIN_ID=42` ve `ROS_LOCALHOST_ONLY=1` eklendi.
- Yol ve şerit çizgileri 100 metreden 1000 metreye uzatıldı.
- Ctrl+C ile kapanış sırasında geçersiz ROS context'i üzerinden yayın yapma
  hatası düzeltildi.
- Gazebo testi sırasında şerit sapmasının yaklaşık 27 Hz yayımlandığı ve
  robotun 0.35 m/s hızla şerit merkezinde ilerlediği doğrulandı.
- 20 metre merkez yarıçapına ve 4 metre şerit genişliğine sahip, 96 segmentli
  dairesel kapalı pist oluşturuldu.
- Pist üretimini tekrarlanabilir hale getiren `generate_lane_world.py` aracı
  eklendi.
- Virajdaki iç şeridin algılanabilmesi için kamera ROI'si genişletildi.
- Kapalı pist testinde robotun pist yarıçapını koruyarak virajı takip ettiği
  doğrulandı; örnek ölçümde şerit sapması -2 piksel olarak gözlendi.
- Kamera işleme yükündeki kısa yayın gecikmeleri nedeniyle gereksiz duruşları
  önlemek için algılama zaman aşımı 1.0 saniyeye çıkarıldı.
- Robot 0.35 m/s hızla yaklaşık 360 saniyede kesintisiz bir tam tur tamamladı
  ve ikinci tura geçti. Tur boyunca ölçülen yörünge yarıçapı yaklaşık 19.69
  metre, tipik şerit sapması -1 ile -3 piksel aralığında kaldı.
- `spawn_y=+1.0` ve `spawn_y=-1.0` başlangıç sapmalarında şeridi yeniden
  merkezleme doğrulandı.
- Dış başlangıçta iç şeridi görebilmek için kamera ROI'si alt yol bandında tam
  genişliğe açıldı.
- Büyük başlangıç hatalarında aşırı dönüşü önlemek için maksimum açısal hız
  0.8 rad/s'den 0.2 rad/s'ye düşürüldü.

## Çalıştırma

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch lane_simulation_pkg lane_simulation.launch.py
```

Simülasyonu durdurmak için launch terminalinde `Ctrl+C` kullanılabilir.

## Kaldığımız nokta

Robot Gazebo kamerasından aldığı görüntüyle kapalı dairesel pistteki iki şerit
çizgisini algılıyor, merkez sapmasını hesaplıyor ve tam turu otonom olarak
tamamlıyor. Merkezden bir metre iç ve dış başlangıçlarda şeridi yeniden
yakaladığı doğrulandı.

## Bilinen durumlar

- Mevcut kapalı pist sabit yarıçaplı tek bir sol viraj içeriyor; farklı viraj
  yarıçapları ve sağ virajlar henüz test edilmedi.
- `robot_state_publisher`, kök bağlantıdaki inertia için KDL uyarısı veriyor.
  Bu uyarı mevcut sürüşü engellemiyor.
- Gazebo Classic kullanım ömrü uyarısı gösteriyor. Şimdilik ROS 2 Humble ile
  Gazebo Classic kullanılmaya devam ediliyor.
- Telif testleri paket şablonundaki lisans alanları tamamlanmadığı için
  atlanıyor.

## Sonraki adımlar

1. Yapay şerit kaybı ve yeniden yakalama senaryosunu test etmek.
2. Daha büyük başlangıç sapmalarının güvenli sınırını belirlemek.
3. Farklı yarıçaplı sol/sağ virajlar için algılama ve kontrolcü ayarlarını
   doğrulamak.
4. Kök link inertia uyarısını dummy/base footprint link ile gidermek.
5. Paket açıklaması, lisans bilgileri ve ana `README.md` dokümantasyonunu
   tamamlamak.
6. Otomatik launch/integration testleri eklemek.
