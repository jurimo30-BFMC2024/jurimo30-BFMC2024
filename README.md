# Dokumentacija sistema - BFMC2024 Brain

## 1. Uvod

Ovaj projekat predstavlja softversku arhitekturu za upravljanje autonomnim vozilom u okviru Bosch Future Mobility Challenge-a. Sistem je razvijen u Python-u i koristi višestruke procese i niti za obradu slike, komunikaciju sa hardverom, razmenu podataka sa drugim vozilima i serverima, kao i za vizualizaciju i nadzor putem web interfejsa.

---

## 2. Arhitektura sistema

Sistem je organizovan kao skup nezavisnih procesa, gde svaki proces ima jasno definisanu odgovornost (npr. detekcija traka, detekcija objekata, komunikacija sa kamerom, serijska komunikacija sa mikrokontrolerom, dashboard, itd). Svaki proces može imati jednu ili više niti (threads) koje paralelno izvršavaju zadatke unutar procesa.

### 2.1. Glavne komponente

- **Gateway**: Centralni proces za razmenu poruka između svih ostalih procesa. Implementira prioritizovane redove (Critical, Warning, General, Config) i omogućava asinhronu komunikaciju.
- **Dashboard**: Web server (Flask + SocketIO) za monitoring i upravljanje sistemom, uključuje i frontend (Angular).
- **Camera**: Proces za akviziciju slike sa kamere i njeno prosleđivanje drugim procesima.
- **LaneDetect / ObjectDetection**: Procesi za obradu slike i detekciju traka, objekata i saobraćajnih znakova.
- **SerialHandler**: Proces za komunikaciju sa mikrokontrolerom (NUCLEO) putem serijskog porta.
- **Semaphores / TrafficCommunication**: Procesi za prijem i slanje podataka o semaforima, drugim vozilima i lokaciji (UWB).
- **Core**: Centralni proces za upravljanje režimima vožnje (manual/auto) i logikom vozila.
- **VideoStream**: Proces za streaming video sadržaja za monitoring i debug potrebe.

---

## 3. Procesi i niti

### 3.1. WorkerProcess i ThreadWithStop

Svi procesi nasleđuju apstraktnu klasu `WorkerProcess`, koja obezbeđuje osnovnu logiku za pokretanje, zaustavljanje i upravljanje nitima. Svaka nit nasleđuje klasu `ThreadWithStop`, koja omogućava bezbedno zaustavljanje niti putem internog flag-a.

#### Primer:
- `processLaneDetect` pokreće nit `threadLaneDetect` koja obrađuje slike i šalje rezultate detekcije.
- `processObjectDetection` pokreće nit `threadObjectDetection` za detekciju objekata i znakova.

### 3.2. Upravljanje životnim ciklusom

- Svaki proces u metodi `_init_threads` kreira i startuje svoje niti.
- Zaustavljanje procesa (`stop`) propagira signal svim nitima da završe rad i zatvore resurse.
- Glavni program (`main.py`) pokreće sve procese kao demone i brine o njihovom zaustavljanju pri gašenju sistema.

---

## 4. Obrada slike - Arhitektura i Pipeline

### 4.1. Opšta arhitektura obrade slike

Sistem za obradu slike se sastoji od nekoliko nezavisnih procesa koji paralelno obrađuju video stream sa kamere:

#### Procesi za obradu slike:
- **processLaneDetect**: Detekcija i praćenje traka na putu
- **processObjectDetection**: Detekcija objekata, saobraćajnih znakova i prepreka
- **processVideoStream**: Streaming obrađenog video sadržaja za monitoring

### 4.2. Pipeline obrade slike

1. **Akvizicija slike**: Kamera šalje raw slike kroz `serialCamera` poruke
2. **Preprocesiranje**: Konverzija u grayscale, filtriranje, normalizacija
3. **Detekcija**: Specifični algoritmi za detekciju traka i objekata
4. **Post-procesiranje**: Filtriranje rezultata, praćenje kroz vreme
5. **Vizualizacija**: Crtanje rezultata na slici za debug i monitoring
6. **Komunikacija**: Slanje rezultata drugim procesima

### 4.3. Detekcija traka (Lane Detection)

#### Klasa `LaneDetector`

Implementira robusnu detekciju traka sa sledećim karakteristikama:

**Ključne komponente:**
- **Region of Interest (ROI)**: Definisan trapezoidni region koji pokriva put
- **Istorija linija**: Koristi deque strukture za filtriranje kroz vreme
- **Ekstrapolacija**: Produžavanje detektovanih linija do ciljne tačke
- **Adaptivna klasifikacija**: Dinamičko klasifikovanje leve i desne trake

**Algoritam:**
1. Aplikacija ROI maske na edge-detektovanu sliku
2. Hough transform za detekciju linija
3. Filtriranje linija po nagibu, poziciji i dužini
4. Klasifikacija u leve i desne trake
5. Ekstrapolacija do ciljne tačke (55% visine slike)
6. Čuvanje u istoriji za stabilnost

**Parametri filtriranja:**
- `min_slope_threshold = 0.2`: Minimalni nagib za validne trake
- `min_lane_line_length = 15`: Minimalna dužina linije u pikselima
- `min_line_pixels = 5`: Minimalan broj piksela za validnu liniju

#### Klasa `ImagePreProcessing`

Optimizovan pipeline za preprocesiranje slika:

**Koraci obrade:**
1. **Konverzija u grayscale**: RGB → Grayscale
2. **ROI primena**: Uklanjanje nepotrebnih delova slike
3. **Median blur**: Uklanjanje šuma (kernel 7x7)
4. **Histogram normalizacija**: Poboljšanje kontrasta koristeći Numba optimizaciju
5. **Gamma korekcija**: Poboljšanje svetlih oblasti (gamma=7)
6. **Binarizacija**: OTSU threshold za crno-bele ivice
7. **Thinning**: Zhang-Suen algoritam za stanjivanje linija

**Optimizacije:**
- **Numba JIT kompajliranje**: ~3.5x ubrzanje histogram normalizacije
- **LUT tabele**: Precomputed gamma korekcija
- **In-place operacije**: Minimizovanje kopiranja memorije
- **ROI pre-filtriranje**: Rana primena maske za smanjenje obrade

### 4.4. Detekcija objekata (Object Detection)

Proces `processObjectDetection` je odgovoran za:
- Detekciju saobraćajnih znakova
- Prepoznavanje prepreka na putu
- Detekciju drugih vozila
- Klasifikaciju objekata po važnosti

### 4.5. Video streaming

#### Klasa `VideoGridStreamer`

- **Grid layout**: Organizuje multiple video stream-ove u grid format (2x1)
- **Web interface**: HTTP server na portu 4201 za real-time monitoring
- **Debug overlay**: Prikazuje rezultate detekcije direktno na video stream-u

---

## 5. Komunikacija između procesa

### 5.1. Poruke i redovi

Komunikacija između procesa se vrši putem multiprocessing Queue objekata, podeljenih po prioritetima:
- **Critical** – kritične poruke (npr. sigurnosni alarmi)
- **Warning** – upozorenja
- **General** – standardna razmena podataka (npr. slike, senzorski podaci)
- **Config** – konfiguracione poruke (npr. subscribe/unsubscribe)

Svaka poruka ima sledeću strukturu:
- `Owner` – izvor poruke
- `msgID` – identifikator poruke
- `msgType` – tip poruke
- `msgValue` – vrednost/korisni podaci

### 5.2. Gateway i pretplate

Proces `Gateway` vodi evidenciju o pretplatama (subscribe/unsubscribe) i prosleđuje poruke odgovarajućim procesima/nitima. Svaki proces može se pretplatiti na određene tipove poruka i dobija ih putem Pipe ili Queue objekata.

#### Mehanizam:
- Kada proces želi da prima određene poruke, šalje "subscribe" poruku na `Config` red.
- Gateway ažurira internu mapu pretplata i prosleđuje poruke samo relevantnim procesima/nitima.
- Poruke se mogu isporučivati po FIFO principu ili samo poslednja (LastOnly), u zavisnosti od potrebe.

---

## 6. Primer toka podataka - Obrada slike

### 6.1. Osnovna obrada

1. **Kamera** šalje raw sliku putem `serialCamera` poruke na `General` red
2. **LaneDetect proces** prima sliku i:
   - Preprocesira sliku (grayscale, filtriranje, edge detection)
   - Detektuje trake koristeći Hough transform
   - Filtrira i klasifikuje leve/desne trake
   - Ekstrapolira pozicije traka do ciljne tačke
   - Šalje rezultate kao `laneDetection` poruku
3. **ObjectDetection proces** prima istu sliku i:
   - Detektuje objekte i saobraćajne znakove
   - Klasifikuje po tipovima i važnosti
   - Šalje rezultate kao `objectDetection` poruku
4. **VideoStream proces** kombinuje originalne slike sa overlay-ima za debugging

### 6.2. Tok za upravljanje vozilom

1. **Core proces** prima rezultate detekcije traka i objekata
2. Koristi pozicije traka za kalkulaciju steering angle-a
3. Koristi detekciju objekata za kontrolu brzine i sigurnosne manevre
4. Šalje komande mikrokontroleru putem **SerialHandler** procesa

### 6.3. Monitoring i debug

1. **Dashboard** prima sve debug informacije
2. **VideoStream** omogućava real-time pregled obrađenih slika
3. Debug linije i overlay-ji prikazuju:
   - ROI regione
   - Detektovane trake sa pozicijama
   - Reference linije za steering
   - Centar između traka
   - Offset linije za navigaciju

---

## 7. Upravljanje i monitoring

- Dashboard omogućava vizuelni prikaz svih relevantnih podataka, kao i slanje komandi sistemu.
- Svaki proces vodi sopstveni log i može biti pokrenut/zaustavljen nezavisno.
- Sistem je modularan – nove komponente se lako dodaju kroz šablone (vidi `newComponent.py`).
- Video streaming omogućava real-time monitoring obrade slike na portu 4201.

---

## 8. Performanse i optimizacije

### 8.1. Optimizacije obrade slike

- **Numba JIT kompajliranje**: Kritični delovi koda optimizovani za brzinu
- **LUT tabele**: Precomputed lookup tables za gamma korekciju
- **In-place operacije**: Minimizovanje alokacije memorije
- **ROI pre-filtriranje**: Rana aplikacija region-of-interest maske
- **OpenCV optimizacije**: Korišćenje efikasnih OpenCV funkcija

### 8.2. Memorijsko upravljanje

- **Ponovna upotreba bafera**: Minimizovanje kreiranja novih array-eva
- **Efikasne strukture podataka**: Deque za istoriju linija
- **Memory mapping**: Za velike slike i video stream-ove

---

## 9. Zaključak

Ova arhitektura omogućava robusnu, skalabilnu i lako proširivu platformu za razvoj i testiranje softvera za autonomna vozila. Jasna podela odgovornosti, višestruki procesi i niti, kao i centralizovana razmena poruka, omogućavaju efikasnu obradu podataka i visok stepen paralelizma.

Sistem za obradu slike je posebno optimizovan za real-time performance sa naprednim algoritmima za detekciju traka i robusnim pipeline-om za preprocesiranje.

Za detalje o implementaciji pojedinačnih procesa i niti, pogledati odgovarajuće module u `src/` direktorijumu.

---

## 10. Korisni linkovi

- [BFMC dokumentacija](https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/)
- [Primer voznje na YouTube](https://youtu.be/g9yjqosDLSk?si=2U75NiC9smB4sBtp)

---
