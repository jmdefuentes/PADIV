#!/usr/bin/perl

# Put here your path to ns 2.34
$nspath = '/home/.../ns-allinone-2.34/ns-2.34';


#Running script varying the probability to take a coupon, the dissemination distance and the size of the coupon list. 
foreach $probTake (0.2, 0.5, 0.857) {
  foreach $distance (300, 600, 900){
    foreach $couponListSize (1,15,30,60){
	$trname = sprintf ("V-10k-200-prob%.3d-dist%.3f-cad%.1d.trace", $probTake, $distance, $couponListSize);
        print $trname;
	system("$nspath/ns $nspath/simulationScript.tcl vaad $trname $probTake $distance $couponListSize Mac/802_11");    
      }
   }
}

