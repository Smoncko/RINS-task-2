# RINS-task-2


### Postransko:
- narediti da say_hi() deluje tudi če zaženemo iz drugje kot ~/ros_ws
- levo zgoraj (ob koncu predpisane poti) se robot zatakne. Spremenit bi bilo treba severity levels ste al kaj ze, pomoje nekaj takega. Da si bo pac upal it skoz.
- če je v desnem koridorju in mora it na base position se tudi rado zatakne. Sprememba severity levels v launchu bi znala pomagat. Ali pa nek manever reševanja, če je predolgo statičen.
- spreminjanje hitrosti (ima prehitrost vpliv na zaznavo? Če nima, bi šli morda lahko hitreje? Zdi se, da bi se ta max_velocity dal v config/nav2.yaml naštimat)
- se da narediti, da pri navigiranju pose objectu ne damo fi-ja, in pač pusti robota v katerikoli smeri je pac obrnjen ko pride do ciljne točke? In ga mi potem recimo obrnemo za 360, da pogledamo za obraze, in gre potem naprej - ker zdaj se ze toliko obraca, da ga je prav wasteful it 360 obracat.
- either way, lahko bi rocno dodali obracanja za vsako mesto postanka posebej in bi se actually obrnil proti stenam, ne vseh 360.