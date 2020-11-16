# Wynik

* Najlepszy: 51.67%
* Najgorszy: 49.26%

Średnio około 50%

# Opis algorytmu

Algorytm ma kilka przypadków. W kolejności precedensu:

1. Gdy jesteśmy za blisko ściany z przodu (odczyt z czujnika so3 lub so4 jest mniejszy od pewnej wartości).

Wykorzystywany jest kontroler PD, żeby na podstawie dystansu do ściany z odczytu z czujnika ostro skręcić w prawo (im bliżej ściany, tym ostrzej)

2. Gdy dystans z dowolnego przedniego czujnika (so0, so1, so2, so3, so4) jest mniejszy od pewnej wartości.

To jest przypadek "bazowy", gdy podążamy wzdłóż ściany. Wykorzystywany jest kontroler P, używajacy różnicy odległości z czujnika i różnicy pomiędzy bocznymi odczytami do wysterowania.
Jeżeli odczyt z czujnika so0 jest za duży, to czytany jest so1, a nasteępnie przemnażany przez pewien cosinus tak, zeby wyglądało to jak gdyby odczyt był z so0, itd. z innymi czujnikami, podobnie z tyłu robota.

3. Gdy żaden z przednich czujników nie rejestruje sensownej wartości, wykonywany jest skręt w lewo.


