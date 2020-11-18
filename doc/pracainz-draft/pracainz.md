 [Strona tytułowa]
 [Spis treści]
 [Słownik skrótów i pojęć]
AGD - Artykuły Gospodarstwa Domowego
IoT - Internet Rzeczy (od ang. Internet of Things)
PC - Komputer Osobisty (ang. Personal Computer)
Smart - ang.
Intelligent - ang. Inteligentny
Firmware - oprogramowanie działające na mikrokontrolerze
Arduino - kategoria płytek drukowanych z mikrokontrolerami, również środowisko służące do pisania, kompilacji i wgrywania kodu na te płytki
STM32duino - nieoficjalna implementacja środowiska kompilacji (bibliotek) Arduino dla płytek z mikrokontrolerami STM32
ToF - Czas Lotu (ang. Time of Flight), tutaj w odniesieniu do rodzaju sensora laserowego mierzącego dystans za pomocą pomiaru czasu jaki upływa od wysłania impulsu świetlnego do odebrania jego odbicia od obiektu


# 1. Wprowadzenie
## 1.1 Wstęp
Wraz z rozwojem technologii i wciąż rosnącym popytem na nowinki elektroniczne łatwo jest zaobserwować jak z biegiem czasu pewne rozwiązania adaptowane są do użytku codziennego. Sprzęt niegdyś będący jedynie drogą ciekawostką staje się akcesorium bez którego coraz trudniej jest wyobrazić sobie normalne życie. Tak było z komputerami osobistymi, płatnościami elektronicznymi, dostępem do Internetu, następnie do listy dołączyły smartfony, hulajnogi elektryczne czy douszne słuchawki bluetooth. Coraz popularniejsze stają się także rozwiązania IoT z zakresu AGD - inteligentne pralki, lodówki, odkurzacze. I to właśnie ostatnie z rozwiązań szczególnie przykuł uwagę autora. Podczas gdy wiele sprzętu reklamowanego z wykorzystaniem haseł takich jak "smart" czy "intelligent" z inteligencją nie ma zbyt wiele wspólnego, to ta granica zdaje się coraz mocniej zacierać - szczególnie w przypadku autonomicznych jednostek stworzonych w celu konserwacji powierzchni płaskich. I tutaj nasuwa się pytanie - Dlaczego? Otóż wiele z wymienionych wyżej urządzeń AGD realizuje pojedyncze, wyznaczone, stosunkowo proste zadania a nowoczesnym dodatkiem do tego ma być łączność z siecią, zdalna obsługa z poziomu aplikacji czy duży, kolorowy wyświetlacz pokazujący aktualną pogodę. Oczywiście inteligentne odkurzacze również oferują podobne udogodnienia, jednak nie to wyróżnia je na tle innych akcesoriów. Tym czynnikiem jest złożoność, na pierwszy rzut oka prostego, zadania które realizują. Na drodze do budowy samodzielnie odkurzającego robota stoi wiele przeszkód związanych z ładowaniem, mapowaniem, nawigacją, omijaniem obiektów stojących na wyznaczonej trasie, radzeniem sobie ze zmieniającym się otoczeniem. W tej pracy przedstawione zostanie autorskie podejście do jednego z tych problemów, przy którym zostaną wykorzystane zarówno własnoręcznie sporządzone jak i gotowe, publicznie dostępne rozwiązania. Tym problemem jest tworzenie mapy pokoju.

## 1.2 Cel i zakres pracy

```usunto?
Celem jest stworzenie działającego systemu składającego się z autonomicznego robota (platformy mobilnej) połączonego z aplikacją na komputerze
stacjonarnym lub laptopie. Robot ten ma za zadanie samodzielnie poruszać
się po pomieszczeniu i skanować je w poszukiwaniu przeszkód (ścian, foteli, nóg krzeseł, stołów itp.), a pozyskane dane przesyłać do komputera. W
aplikacji z danych zebranych z otoczenia robota ma powstać dwuwymiarowa
mapa pomieszczenia opisująca je na poziomej płaszczyźnie ok. 10 cm nad
poziomem podłogi.
• zaprojektowanie i zbudowanie jeżdżącego robota wyposażonego w sensor odległości i moduł Bluetooth do komunikacji z komputerem
1
• implementacja algorytmu pozwalającego na autonomiczne poruszanie
się robota po pomieszczeniu
• napisanie aplikacji rysującej mapę pomieszczenia działającej na komputerze lub laptopie
```

Celem niniejszej pracy jest stworzenie mobilnej platformy skanującej (dalej określanej mianem robota), która będzie w stanie samodzielnie poruszać się po pomieszczeniu, omijać przeszkody, i przesyłać do centralnej jednostki sterującej (komputera PC) dane pozwalające na utworzenie dwuwymiarowej mapy pomieszczenia w którym się znajduje.

W zakres pracy wchodzi:
- Budowa robota zdolnego do poruszania się oraz zbierania danych z otoczenia niezbędnych do realizacji pozostałych założeń
- Napisanie oprogramowania sterującego na PC
- Napisanie oprogramowania działającego na robocie (Firmware)
- Zestawienie stabilnej komunikacji pomiędzy PC a robotem
- Implementacja algorytmu pozwalającego na autonomiczną jazdę robota i omijanie przeszkód
- Implementacja/Wykorzystanie algorytmu nawigacji zliczeniowej 
- Implementacja/Wykorzystanie algorytmu pozwalającego na stworzenie mapy pomieszczenia


## 1.3 Koncepcja projektu
Po pierwsze robot powinien posiadać zasilanie akumulatorowe - w przeciwnym wypadku wymagałby podania zasilania za pośrednictwem przewodu, co byłoby niepraktyczne. Aby zasilania starczyło na jak najwięcej czasu dobrze byłoby, aby nie wykorzystywał zgromadzonej energii na obliczenia, które mogą zostać wykonane po stronie aplikacji go kontrolującej. Z tego względu do kontroli peryferiów platformy został wykorzystany mikrokontroler.

Aby móc sporządzić mapę potrzebne będą lokalizacje przeszkód w postaci punktów reprezentowanych na dwuwymiarowej płaszczyźnie. Taki punkt można obliczyć znając odległość i kierunek do przeszkody względem robota - potrzebuje on więc  sensorów dzięki którym będzie w stanie tą odległość zmierzyć. Istnieją gotowe rozwiązania korzystające z lasera obracającego się na podstawie , jednak są na tyle kosztowne, że zdecydowano się na tańszą alternatywę. Sensor odległości zamontowany na  "wieżyczce" obracającej się za pomocą serwomechanizmu  w zakresie od 0 do 180 stopni jest wystarczający do realizacji tego zadania.

Aby pozycja wcześniej wspomnianych punktów była relatywna do pomieszczenia platforma powinna realizować zadanie nawigacji zliczeniowej (odometrii). Dzięki temu możliwe jest oszacowanie aktualnej pozycji i skierowania robota w przestrzeni względem jego pozycji startowej. Dopiero korzystając z tych danych połączonych z dystansem i kierunkiem do przeszkody można umieścić ją na mapie. Do realizacji tej funkcji robot został wyposażony w mechaniczne enkodery, po jednym na lewą i prawą stronę.

Dla mniejszego poślizgu oraz łatwego pokonywania niewielkich, nieznaczących przeszkód takich jak przewody, listwy zasilające, dywaniki oraz dla ułatwienia obliczeń związanych z odometrią robot porusza się na gąsienicach.

Należy również ustalić w jakim języku programowania napisane będą programy - wszakże w zależności od środowiska programista powinien spodziewać się innych możliwości i ograniczeń. Ze względu na możliwość szybkiego prototypowania i czytelność kodu do napisania oprogramowania PC wybrany został język Python. Oprogramowanie robota zostało napisane w języku C++.


## 1.4 Przegląd technologii
Nie jest tajemnicą że inspiracją do tego projektu była rozwijająca się już od ponad dwóch dekad branża robotycznych odkurzaczy. Oczywiście nie jest to jedyna sfera w której implementowane są algorytmy autonomicznej jazdy i mapowania. Nie mniej jednak podczas poszukiwań to na tym autor skupił swoją uwagę stąd ostatecznie wybrana technologia jest jedną z wykorzystywanych właśnie w tym segmencie.

Warto jednak nadać pewien rys historii rozwoju tej branży i technologii które były stosowane.

```
- losowe odbijanie się za pomocą - tu sprawdzić czy sensorów ultradzw. czy bumperow czy IR
- SLAM (LIDAR)
- VSLAM (kamera)
- hybrydowe
```

# 2. Projekt
## Oprogramowanie PC
### Struktura oprogramowania
```
Dac jakies UML itede
```
### Autonomiczna jazda
```
tu napisać o swoim algorytmie, dac rysunek stref
```
### Skan otoczenia
```
tu napisac o bazowym mapowaniu pkt na plaszczyzne
o swojej implementacji ze scorem
nastepnie o tym co moznaby dalej (translacja korekta)
ale nie zostalo zrobione bo bylo slabe i to wymaga filtrow czasteczek
i ze jest cos takiego jak cartographer od googla
ale zdecydowano ze gmapper jest spoko i czemu nie gmapper

```

### Odometria
#### Kalibracja magnetometru
```
koniecznie daj screenshoty
opisz hard i soft iron offset
powiedz o filtrze kalmana

```
#### UMBenchmark[przypis]
```
tu dac fotki koniecznie
jakies rysunki nabazgrane z kwadratami
i powiedziec ze sie nie pokrywalo najlepiej
dac tabelke z pozycjami
pokazac obliczenia center of gravity itd
ej wyszlo ~1% czyli to bez sensu
robot juz jezdzi dobrze
to zasługa wielkiego myśliciela autora tego tekstu
oraz komitetu centralnego
a jakże
```
## Firmware
```
powiedzieć 
```
## Schemat budowy fizycznej
```
powiedz co sie zmienialo, jaki jest problem z gasienicami
wyjasnij ze gasienice upraszczamy do 2 kolek
```

## Schemat elektroniczny
```
wspomnij o sensorach ultradzw oraz sharp IR
```

# 3. Ewaluacja

# 4. Podsumowanie
## 4.1 Ocena własna projektu
## 4.2 Co można było zrobić lepiej-lepiej nie
## 4.3 Wnioski
### Odometria sama w sobie jest do kitu
### Nie można polegać na sensorach (na pewno nie na jednym)

[Literatura]
[Dodatki]
