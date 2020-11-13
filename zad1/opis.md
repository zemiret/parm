# Wynik

* Segment 1: 91.76%
* Segment 2: 87.26%
* Segment 3: 89.72%
* Segment 4: 84.71%

Average: 88.36%

# Opis algorytmu

Algorytm używa sensora prawego koła, aby wyliczać czy robot ma dalej jechać czy już nie.

Dla kolejnych boków (i=0,1,2,3) jako bazowy dystans przyjęte jest
```
next_stop_dm = side_dist_dm * (i + 1) + wheel_dist_modifier + wheel_modifier - turn_dist_modifier
```
czyli bazowo 

	* 20dm na długość boku +
	* odległość między kołami / 2 (żeby mieć przestrzeń na skręty) +
	* wheel_modifier (dodawana po przejechaniu pierwszego boku średnica koła) -
	* turn_dist_modifier (ponieważ musimy odjąć to ile się "nakręciliśmy" kołem)

natomiast odległość każdorazowego skrętu jest wyliczona jako 1/4 obwodu koła o średnicy 300mm (odległości między kołami robota).
Robot również nie skręca na ostatniej prostej, co pozwala uzyskać dodatkowe ułamki procenta.
Ponadto, czasem dodawane lub odejmowane jest pewna wartość od odpowiednich dystansów.
Jest to czysto eksperymentalnie wyznaczona wartość bez specjalnego uzasadnienia.

