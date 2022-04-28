# WALL_E
IELET1002 Smart City prosjekt gruppe 22

I dette Smart City prosjektet har roboten WALL.E fått i opprag å rydde opp byen E-Ville, slik at menneskene en dag kan flytte tilbake. WALL.E tjener penger ved å pante søppel den finner, deretter bruker den pengene på å lade opp batteriene sine. Dette er altså en viktig oppgave som er avgjørende for at menneskene skal kunne reise tilbake til jorden.

WALL.E er bygget på ATmega32U4 miktrokontrolleren, og tar også i bruk en ESP32 mikrokontroller for kommunikasjon med menneskenes romstasjon. Romstasjonen drives av en Raspberry Pi, og kommuniserer med WALL.E ved hjelp av en ESP32.

Infrastrukturen på bakken baserer seg på en motorvei rundt byen, av typen "EL-teip". Roboten kjører rundt i byen og leter etter søppel. Når søppel blir funnet, kjører roboten ut til motorveien og deretter ut av byen til en miljøstasjon der roboten kan pante søppelet sitt, og lade batteriene.
