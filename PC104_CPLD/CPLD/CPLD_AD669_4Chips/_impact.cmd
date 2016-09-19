setMode -bs
setMode -bs
setCable -port auto
Identify 
identifyMPM 
assignFile -p 1 -file "F:/chendaixie/OpticalCouple_TD/CPLD/CPLD_AD669_4Chips/AD669_4Chips.jed"
Program -p 1 -e -defaultVersion 0 
Program -p 1 -e -defaultVersion 0 
Program -p 1 -e -defaultVersion 0 
Program -p 1 -e -defaultVersion 0 
setMode -bs
deleteDevice -position 1
setMode -ss
setMode -sm
setMode -hw140
setMode -spi
setMode -acecf
setMode -acempm
setMode -pff
setMode -bs
