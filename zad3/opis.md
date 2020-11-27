# Wynik

W miarę konsystentne 1578.715 m

# Opis algorytmu

Zasada działania algorytmu opiera się na 2 kontrolerach:

* niskopoziomowy - ReactiveController
* wysokopoziomowy - DecisiveController

ReactiveController odpowiada za utrzymywanie poprawnego skrętu oraz prędkości 
i wystawia API do wyżej poziomowego kontrolera.
Kontroler, który reguluje kąt kierownicy działa w 2 trybach.
Pierwszy tryb, to tryb "trzmaj się zadanego x", który jest wykorzystywany
na środkowym i prawym pasie. Drugi tryb, to tryb "trzymaj się lewej barierki",
który jest wykorzystywany na lewym pasie. W ten sposób rozwiązany jest
problem skręcającej drogi.
Kontroler utrzymujący prędkość zmienia ją liniowo do odczytu odległości z przedniego czujnika.

DecisiveController stara się, żeby samochód zawsze utrzymać na lewym pasie,
o ile to możliwe. Czyli:
* jeżeli przed nami czysto i jesteśmy na lewym pasie - utrzymuj
* jeżeli nie jesteśmy na lewym pasie, a da się skręcić na lewo - skręć
* jeżeli przed nami nie jest czysto, ani nie możemy skręcić w lewo, a możemy skręcić w prawo - skręć

Jeżeli żaden z tych przypadków się nie aplikuje - czyli przed nami coś jest i
nie możemy wykonać manewru ani w lewo, ani w prawo,
to niższa wartstwa - ReactiveController powinna poprawnie zwolnić.

Jest też SensorManager, który wystawia pewne API nad czujnikami.

