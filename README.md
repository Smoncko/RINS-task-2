# RINS-task-2

### Spreminjanje parametrov stene:
V config/nav2.yaml greš pod:
local_costmap:   local_costmap:    ros_parameters:   inflation_layer:
in spremeniš 
cost_scaling_factor (originalno 4.0)
inflation_radius  (originalno 0.45)

## To do:
- text to speech
- detektiranje cilindrov (z barvo) robustifikacija, subscription, prependanje teh goalov
- Detektiranje krogov (z barvo, pa ne flat krogov torej preverit depth na krogu in v sredini in videt če sta različna (pazi na infinities in take stvari ko v ozadju ni stene ampak air)), subscription, prependanje teh goalov - če je zelen, potem še prependanje goala servoing mehanizma
- servoing mehanizem, kjer gledamo za krog na tleh in probamo bit celi noter.
Ideji asistenta na vajah:
- z zgornjo kamero (na robotski roki) pogledaš blob in s cv2 dobiš njegov center (blob je ta lunca ki jo omejujeta t in vidni del circlea - ta notranjost). In se obrneš proti temu centru in potuješ v tisto smer. In ko je blob dovolj simetričen in ko je center bloba dost v redu, ali pa ko je dost thin, takrat si zmagal.
- pogledaš vidni ring in fittaš hough transform, da dobiš center kroga. Ker pravokotno na tla gledaš pomoje lahko celo circle fittaš, ne elipse. In lepo dobiš center na image coordinates. Potem pa imamo narejeno že nmeko transformacijo iz image v world coordinates. In se premikaš proti centru dokler napaka ni dovolj majhna. Ampak ta transformacija je nekoliko nerobustna, zato asisten predlaga prvo opcijo.

### Postransko:
- narediti da say_hi() deluje tudi če zaženemo iz drugje kot ~/ros_ws
- levo zgoraj (ob koncu predpisane poti) se robot zatakne. Spremenit bi bilo treba severity levels ste al kaj ze, pomoje nekaj takega. Da si bo pac upal it skoz.
- če je v desnem koridorju in mora it na base position se tudi rado zatakne. Sprememba severity levels v launchu bi znala pomagat. Ali pa nek manever reševanja, če je predolgo statičen.
- spreminjanje hitrosti (ima prehitrost vpliv na zaznavo? Če nima, bi šli morda lahko hitreje? Zdi se, da bi se ta max_velocity dal v config/nav2.yaml naštimat)
- se da narediti, da pri navigiranju pose objectu ne damo fi-ja, in pač pusti robota v katerikoli smeri je pac obrnjen ko pride do ciljne točke? In ga mi potem recimo obrnemo za 360, da pogledamo za obraze, in gre potem naprej - ker zdaj se ze toliko obraca, da ga je prav wasteful it 360 obracat.
- either way, lahko bi rocno dodali obracanja za vsako mesto postanka posebej in bi se actually obrnil proti stenam, ne vseh 360.