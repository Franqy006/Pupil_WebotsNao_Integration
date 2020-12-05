# Pupil_WebotsNao_Integration

## aktualna zawartość:
### skrypty:
  * marker.py - sterowanie za pomocą markerów
  * normalized.py - sterowanie manualne
  * pupil_zmq_ros_pub.py - łączenie z urządzeniem po tcp i publikowanie wiadomości po rosie
  * robotNameReader.py - odczytuje nazwę robota z webots, zapisuje ją jako rosparam
  * start_pupil.py - uruchamia pupil_capture
### launch:
  * markers.launch - uruchamia powyższe skrypty ze sterowaniem markerami
  * normalized.launch - uruchamia powyższe skrypty ze sterowaniem manualnym
### msg:
  * Typy wiadomości skopiowane z zewnętrznej biblioteki
  
  
  
## elementy do zrobienia:
* szczegółowy opis, jak uruchomić z webots + zewn. paczki wymagane
* markery (grafiki)
* (done)zbędne importy w skryptach
* (done)zmiana nazewnictwa na angielskie
* (done)wykorzystywanie wiadomości z tego repozytorium, nie pupil_ros_plugin
* (done)na razie zostawiłem zakomentowane zbędne części pupil_zmq_ros_pub.py
