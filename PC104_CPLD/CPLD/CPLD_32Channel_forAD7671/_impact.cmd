setMode -bs
setMode -bs
setCable -port auto
Identify 
identifyMPM 
assignFile -p 1 -file "F:/chendaixie/OpticalCouple_TD/CPLD/CPLD_32Channel_forAD7671/CPLD_32Channel_forAD7671.jed"
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
