#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
=====================================================================================
                    TEKNİK YAPILANDIRMA AÇIKLAMASI
=====================================================================================
1. Satır (Shebang - #!/usr/bin/env python3):
   - Bu satır, Linux/ROS ortamına bu dosyanın bir "Python 3" betiği olduğunu ve
     uygun yorumlayıcı ile çalıştırılması gerektiğini bildirir.
   - '/usr/bin/env' komutu, sistemdeki Python 3 yolunu dinamik olarak bularak
     kodun farklı bilgisayarlarda (taşınabilirlik) sorunsuz çalışmasını sağlar.
   - Bu satır sayesinde dosya './dosya_adi.py' veya 'rosrun' komutuyla doğrudan
     çalıştırılabilir hale gelir.

2. Satır (Encoding - # -*- coding: utf-8 -*-):
   - Python yorumlayıcısına dosyanın karakter kodlamasının UTF-8 olduğunu belirtir.
   - Bu yapılandırma, kod içerisinde ve yorum satırlarında Türkçe karakterlerin
     (ç, ğ, ş, ö, ü, İ) hata almadan ve bozulmadan kullanılmasını garanti eder.
=====================================================================================

=====================================================================================
"""

#####################################################################################
#####################################################################################
##                                                                                 ##
## PROJE     : ROS VE GAZEBO SİMÜLASYON ORTAMINDA LİDAR SENSÖR TABANLI OTONOM      ##
##             ENGELDEN KAÇAN ROBOT TASARIMI                                       ##
## MİMARİ    : ÇOKLU İŞ PARÇACIĞI (MULTITHREADED) & REAKTİF KONTROL                ##
## PROGRAMCI : ALİ ÖZKAN                                                           ##
##                                                                                 ##
#####################################################################################
#####################################################################################

#####################################################################################
## LİSANS      : CC BY-NC-SA 4.0 (Atıf-Gayriticari-AynıLisanslaPaylaş)             ##
## TELİF HAKKI : 2026, Ali Özkan                                                   ##
## UYARI       : Bu kod ticari amaçlarla kullanılamaz.                             ##
#####################################################################################

"""
=====================================================================================
                                    AÇIKLAMA
=====================================================================================
Bu kod, "Gap Finding" (Boşluk Bulma) ve "Dynamic Velocity" (Dinamik Hız) 
algoritmalarını içerir. Ayrıca kullanıcı deneyimini artırmak için ROS iletişimi 
ve Navigasyon mantığı ayrı iş parçacıklarında (Thread) çalıştırılır.
=====================================================================================

=====================================================================================
"""

# --- KULLANILACAK KÜTÜPHANELER TANITILIYOR ---
import time
import threading
import math
import sys
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# --- GLOBAL DEĞİŞKENLER VE DURUM BAYRAKLARI ---
global scan_data        # Lidar verisini tutan global değişken
global is_navigating    # Robotun hareket durumu (True/False)
global system_active    # Programın ana döngüsünün durumu
global thread_lock      # Veri okuma/yazma çakışmasını önlemek için kilit

scan_data = None
is_navigating = False
system_active = True
thread_lock = threading.Lock()

# --- ROBOT FİZİKSEL PARAMETRELERİ ---
ROBOT_WIDTH = 0.50          # Robot genişliği (Guvenlik payı dahil - metre)
MAX_SPEED_LINEAR = 0.26     # Maksimum dogrusal hiz (m/s)
MAX_SPEED_ANGULAR = 1.82    # Maksimum açısal hız (rad/s)
MIN_STOP_DIST = 0.25        # Acil duruş mesafesi (m)
GAP_THRESHOLD = 0.80        # Bir aralığın "boşluk" sayılması için gereken derinlik (m)

# --- ROS DÜĞÜM BAŞLATMA ---
rospy.init_node('otonom_navigasyon', anonymous=True)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()


# --- MATEMATİKSEL FONKSİYONLAR (Algoritma Detaylari) ---

def calculate_alpha(distance):
    """
    [Proje Denklemi]: Robotun belirli bir mesafedeki (distance) engellerin arasından
    geçebilmesi için gereken minimum açisal genişliği (alpha) hesaplar.
    
    Formül: alpha = 2 * arctan( Robot_Genisligi / (2 * Mesafe) )
    """
    if distance <= 0: 
        return 90.0 # Mesafe 0 ise maksimum açı döndür (Hata koruması)
    
    # Radyan cinsinden hesaplama
    alpha_rad = 2 * math.atan(ROBOT_WIDTH / (2 * distance))
    
    # Dereceye çevirme
    alpha_deg = math.degrees(alpha_rad)
    
    # Güvenlik payı olarak +1 derece eklenir (Tez önerisi)
    return alpha_deg + 1.0


def calculate_dynamic_velocity(front_distance):
    """
    [Proje Denklemi]: Robot engele yaklaştıkça hızı doğrusal değil, üstel (exponential)
    olarak azaltır. Bu sayede robot engellere yaklaşırken daha yumuşak (smooth) durur.
    
    Formul: V = V_max * (1 - exp( -k * (d - d_min) ))
    """
    if front_distance < MIN_STOP_DIST:
        return 0.0
    
    # Hız profili hesabı
    # Katsayı (1.0) robotun frenleme sertliğini belirler
    velocity = MAX_SPEED_LINEAR * (1 - math.exp(-(front_distance - MIN_STOP_DIST)))
    
    # Sınırlandırma (Saturation)
    if velocity > MAX_SPEED_LINEAR:
        velocity = MAX_SPEED_LINEAR
    elif velocity < 0.0:
        velocity = 0.0
        
    return velocity


def get_clean_scan_data(raw_msg):
    """
    Lidar sensöründen gelen ham veriyi temizler.
    1. Sonsuz (inf) değerleri temizler.
    2. Hatalı (NaN) değerleri temizler.
    3. Çok yakın (0.15m altı) gürültü değerlerini filtreler.
    """
    clean_ranges = []
    for r in raw_msg.ranges:
        if math.isinf(r) or math.isnan(r):
            clean_ranges.append(3.5) # Maksimum menzil ata
        elif r < 0.15:
            # Lidar bazen robotun kendi kablosunu gorebilir, bunu engellemek icin
            # cok yakin degerleri "uzak" kabul ediyoruz (Filtreleme).
            clean_ranges.append(3.5) 
        else:
            clean_ranges.append(r)
    return clean_ranges


def find_best_gap(ranges):
    """
    [Gap Finding Algoritması]: Robotun etrafındaki en geniş ve en derin boşluğu arar.
    Sadece sağ veya sola bakmaz; tüm panoramayı tarayarak en güvenli rotayı bulur.
    
    Dönuüş: (Hedef Açı [Radyan], Hedef Mesafe [Metre])
    """
    # Veri işleme kolaylığı için sağ ve sol sektörü birleştiriyoruz.
    # Turtlebot3 Lidar Yapısı: [0-90] Sol, [270-360] Sağ.
    # Birleştirilmiş Panorama: [Sağ Taraf (90 derece)] + [Sol Taraf (90 derece)]
    panorama = ranges[270:360] + ranges[0:90]
    
    max_score = -1          # En iyi boşluk puanı
    best_angle_index = -1   # En iyi boşluğun orta noktası
    current_gap_start = -1  # Boşluk başlangıç indeksi
    
    # Panoramayı baştan sona tara
    for i in range(len(panorama)):
        dist = panorama[i]
        
        # Eğer mesafe eşik değerden büyükse, burası bir "boşluk"tur.
        if dist > GAP_THRESHOLD:
            if current_gap_start == -1:
                current_gap_start = i # Yeni bir bosluk basladi
        else:
            # Bosluk bitti veya engel var
            if current_gap_start != -1:
                # Boslugu analiz et ve puanla
                gap_width = i - current_gap_start
                
                # Heuristic Puanlama: Genislik * Ortalama Derinlik
                # (Genis ve derin bosluklar daha yuksek puan alir)
                avg_depth = sum(panorama[current_gap_start:i]) / gap_width
                score = gap_width * avg_depth
                
                if score > max_score:
                    max_score = score
                    # Boslugun tam ortasini hedef olarak sec
                    best_angle_index = current_gap_start + (gap_width // 2)
                
                current_gap_start = -1 # Sayaci sifirla

    # Eğer hiç uygun boşluk bulunamadıysa (Örn: Çıkmaz Sokak)
    if best_angle_index == -1:
        return None, 0.0
    
    # Bulunan indeksi robota göre açıya çevir
    # Panorama 0..180 arası indeksli. 90. indeks robotun tam önü (0 derece).
    # 0. indeks = -90 derece (Sağ), 180. indeks = +90 derece (Sol).
    target_angle_deg = best_angle_index - 90
    target_dist = panorama[best_angle_index]
    
    return math.radians(target_angle_deg), target_dist


# --- İŞ PARÇACIKLARI (THREADS) ---

def ros_callback_thread(msg):
    """
    ROS'tan gelen Lidar verisini sÜrekli gÜncelleyen callback fonksiyonu.
    Ana döngüden bağımsız çalışır, böylece veri akışı asla kesilmez.
    """
    global scan_data
    with thread_lock:
        scan_data = msg

def navigation_control_thread():
    """
    [ANA BEYİN]: Robotun hareket kararlarını veren thread.
    Kullanıcı 'g' tuşuna bastığında aktifleşir, 's' ile durur.
    Terminal komutlarından bağımsız (Non-blocking) çalışır.
    """
    global is_navigating, system_active, scan_data
    
    rate = rospy.Rate(10) # 10 Hz (Saniyede 10 karar)
    
    print(">>> Navigasyon Kontrol Modülü Yüklendi. (Beklemede...)")
    
    while system_active and not rospy.is_shutdown():
        # Eğer navigasyon aktif değilse veya veri gelmediyse bekle
        if not is_navigating or scan_data is None:
            time.sleep(0.1)
            continue
            
        # Global veriyi kopyala (Thread safe olması için)
        with thread_lock:
            current_scan = get_clean_scan_data(scan_data)
        
        # Ön Güvenlik Mesafesi (Robotun tam önündeki 20 derecelik açı)
        front_distance = min(current_scan[0:10] + current_scan[350:360])
        
        linear_x = 0.0
        angular_z = 0.0
        log_status = ""
        
        # --- DURUM MAKİNESİ (FINITE STATE MACHINE) ---
        
        # DURUM 1: ACİL FREN (Kritik Mesafe)
        if front_distance < MIN_STOP_DIST:
            log_status = "[DURUM]: ACIL DURUM (Geri Kacis)"
            linear_x = -0.10 # Yavasca geri git
            angular_z = 0.0
            
        # DURUM 2: BOŞLUK ANALİZİ VE HAREKET
        else:
            target_angle, target_dist = find_best_gap(current_scan)
            
            if target_angle is None:
                # Gidecek yer yoksa (Koseye sikisti)
                log_status = "[DURUM]: SIKISTI (Donus Yapiyor)"
                linear_x = 0.0
                angular_z = 0.6 # Oldugu yerde don
            else:
                # Dinamik hiz hesapla
                linear_x = calculate_dynamic_velocity(front_distance)
                
                # Hedef aciya dogru oransal (P-Control) donus
                angular_z = target_angle * 1.5 
                
                # Keskin donuslerde savrulmamak icin dogrusal hizi dusur
                if abs(angular_z) > 0.5:
                    linear_x *= 0.5
                
                degree_display = math.degrees(target_angle)
                log_status = f"[DURUM]: SEYIR (Hedef: {degree_display:.1f} dg)"

        # Hesaplanan hızları robota gönder
        move.linear.x = linear_x
        move.angular.z = angular_z
        pub.publish(move)
        
        # Durumu terminale yazdır (Alt alta akar, önceki satırı silmez)
        print(f"{log_status} | Hiz: {linear_x:.2f} m/s | On Mesafe: {front_distance:.2f}m")
        
        rate.sleep()


def stop_robot_emergency():
    """Robotu anında durduran güvenlik fonksiyonu."""
    move.linear.x = 0.0
    move.angular.z = 0.0
    pub.publish(move)
    pub.publish(move) # Garanti olsun diye iki kez gönder
    print("\n!!! ROBOT ACİL DURDURULDU !!!\n")


# --- ANA PROGRAM (MAIN) ---

if __name__ == '__main__':
    try:
        # 1. ROS Lidar Dinleyici Thread Başlat
        # Bu thread arka planda Lidar verisini sürekli günceller.
        ros_thread = threading.Thread(target=rospy.Subscriber, args=('/scan', LaserScan, ros_callback_thread))
        ros_thread.daemon = True # Ana program kapanınca bu da kapansın
        ros_thread.start()
        
        # 2. Navigasyon Mantığı Thread Başlat
        # Bu thread robotun otonom hareketlerini yönetir.
        nav_thread = threading.Thread(target=navigation_control_thread)
        nav_thread.daemon = True
        nav_thread.start()
        
        # 3. Kullanıcı Arayüzü (Ana Thread)
        # Burası sadece kullanıcıdan komut bekler, asla donmaz.
        print("==================================================")
        print("        ENGELDEN KAÇAN ROBOT TASARIMI             ")
        print("==================================================")
        print("          OTONOM NAVİGASYON SİSTEMİ               ")
        print("     [ MULTITHREADED & REAKTİF KONTROL ]          ")
        print("==================================================")
        print(" KOMUT LİSTESİ:")
        print("  g  -> BAŞLAT (Navigasyonu aktif et)")
        print("  s  -> DURDUR (Robotu anında durdur)")
        print("  e  -> CIKIŞ  (Programı ve terminali kapat)")
        print("==================================================")
        
        while not rospy.is_shutdown():
            # input() fonksiyonu main thread'i bloklar ama diger thread'ler calismaya devam eder.
            # Bu sayede robot hareket ederken bile buraya yazi yazabilirsin.
            user_input = input("\nKomut Giriniz (g/s/e) > ").strip().lower()
            
            if user_input == 'g':
                if not is_navigating:
                    print(">>> SİSTEM BASLATILIYOR...")
                    is_navigating = True
                else:
                    print(">>> UYARI: Sistem zaten calisiyor!")
                    
            elif user_input == 's':
                if is_navigating:
                    print(">>> DURDURMA KOMUTU ALINDI!")
                    is_navigating = False
                    stop_robot_emergency()
                else:
                    print(">>> Robot zaten duruyor.")
            
            elif user_input == 'e':
                print(">>> CIKIS YAPILIYOR...")
                is_navigating = False
                system_active = False # Diger thread'leri sonlandir
                stop_robot_emergency()
                time.sleep(1) # Thread'lerin kapanmasi icin bekle
                sys.exit() # Programdan cik
                
            else:
                print(">>> HATA: Gecersiz komut! Lutfen 'g', 's' veya 'e' giriniz.")
                
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        stop_robot_emergency()
        sys.exit()