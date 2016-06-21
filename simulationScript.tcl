#########
# This file was originally provided by Zhengming Li, as the author of VAAD paper. 
# It has been adapted by PADIV authors to meet the new goals pursued by their contribution
# All adapted parts have been marked with UC3M, and they are the only elements whose intellectual property belong to PADIV authors

#This file contains the scripts for running simulation with Leaper
#
#

# This test suite is for validating wireless lans 
# To run all tests: test-all-wireless-lan
# to run individual test:
# ns test-suite-wireless-lan.tcl dsr
# ns test-suite-wireless-lan.tcl wired-cum-wireless
# ns test-suite-wireless-lan.tcl wireless-mip
# ....
#
# To view a list of available test to run with this script:
# ns test-suite-wireless-lan.tcl
#

# 
#
Class TestSuite

Class Test/vaad -superclass TestSuite
# wireless model using GPSR

Class Test/dsr -superclass TestSuite
# wireless model using dynamic source routing

# UC3M

set couponTaken 0
set couponNotTaken 0
set couponSent 0
set commFeedbackDistance 0 
set commFeedbackFinishedChain 0
set commFeedbackNoNeighbours 0 

proc usage {} {
    global argv0
    puts stderr "usage: ns $argv0 <tests> "
    puts stderr "usage: ns $argv0 <tfile> <bint> <bdesync> <bexp> <pint> <pdesync>"
    puts "Valid Tests: dsr gpsr"
    exit 1
}


proc default_options {} {
    global opt
    set opt(chan)	Channel/WirelessChannel
    set opt(prop)	Propagation/TwoRayGround
    set opt(netif)	Phy/WirelessPhy
    set opt(mac)	Mac/802_11
    set opt(ifq)	Queue/DropTail/PriQueue
    set opt(ll)		LL
    set opt(ant)        Antenna/OmniAntenna 
    set opt(x)		10005 ;# X dimension of the topography UC3M CHANGE TO VARY SCENARIO
    set opt(y)		10005 ; #5 ; #5 ;# Y dimension of the topography UC3M CHANGE TO VARY SCENARIO
    set opt(radius)     300  ;# radius of communication (300 m)
    set opt(ifqlen)	50	      ;# max packet in ifq
    set opt(seed)	0.0
    set opt(tr)		temp.rands    ;# trace file
    set opt(lm)         "off"          ;# log movement
    set opt(pint)       8.0            ; #what's this for?
    set opt(pdesync)    0.5
    set opt(bint)       1.0    ;#3
    set opt(bdesync)    0.5
    set opt(bexp)       4.5   ;#13.5
}


# =====================================================================
# Other default settings

set AgentTrace			ON
set RouterTrace			ON
set MacTrace			ON;#OFF
set PhysicalTrace               ON

LL set mindelay_		50us
LL set delay_			25us
LL set bandwidth_		0	;
LL set off_prune_		0	;
LL set off_CtrMcast_		0	;

Agent/Null set sport_		0
Agent/Null set dport_		0

Agent/CBR set sport_		0
Agent/CBR set dport_		0

#Agent/TCPSink set sport_	0
#Agent/TCPSink set dport_	0

#Agent/TCP set sport_		0
#Agent/TCP set dport_		0
#Agent/TCP set packetSize_	1460
#I plan to use UDP agent for data transmission
Agent/UDP set sport_ 0
Agent/UDP set dport_ 0
Agent/UDP set packetsize_ 20  

Queue/DropTail/PriQueue set Prefer_Routing_Protocols    0

# unity gain, omni-directional antennas
# set up the antennas to be centered in the node and 1.5 meters above it
Antenna/OmniAntenna set X_ 0
Antenna/OmniAntenna set Y_ 0
Antenna/OmniAntenna set Z_ 1.5
Antenna/OmniAntenna set Gt_ 1.0
Antenna/OmniAntenna set Gr_ 1.0

# Initialize the SharedMedia interface with parameters to make
# it work like the 914MHz Lucent WaveLAN DSSS radio interface
set RxT_ 3.652e-10 ;#Receiving Threshold which mostly is a hardware feature
set Frequency_  914e+6 ;# Signal Frequency which is also hardware feature

Phy/WirelessPhy set CPThresh_ 10.0
Phy/WirelessPhy set CSThresh_ 1.559e-11
Phy/WirelessPhy set RXThresh_ $RxT_ ;# Receiving Threshold
Phy/WirelessPhy set Rb_ 2*1e6            ;# Bandwidth
Phy/WirelessPhy set freq_ $Frequency_
Phy/WirelessPhy set L_ 1.0

set opt(prop)	Propagation/TwoRayGround  ;#or FreeSpace
set opt(Pt)             300.0 ;# Transmission Power/Range in meters
set opt(AnH)            1.5    ;# antenna height, 1.5 meter
set PI                  3.14159

if { $opt(prop) == "Propagation/TwoRayGround" } {
    set SL_ 300000000.0 ;# Speed of Light
    set lambda [expr $SL_/$Frequency_]   ;# wavelength
    set lambda_2 [expr $lambda*$lambda]  ;# lambda^2
    set CoD_ [expr 4.0*$PI*$opt(AnH)*$opt(AnH)/$lambda] ;# Cross Over Distance
    
    if { $opt(Pt) <= $CoD_ } {;#Free Space for short distance communication
	set temp [expr 4.0*$PI*$opt(Pt)]
	set TP_ [expr $RxT_*$temp*$temp/$lambda_2]
	Phy/WirelessPhy set Pt_ $TP_ ;#Set the Transmissiont Power w.r.t Distance
    } else { ;# TwoRayGround for communicating with far nodes
	set d4 [expr $opt(Pt)*$opt(Pt)*$opt(Pt)*$opt(Pt)]
	set hr2ht2 [expr $opt(AnH)*$opt(AnH)*$opt(AnH)*$opt(AnH)]
	set TP_  [expr $d4*$RxT_/$hr2ht2]
	Phy/WirelessPhy set Pt_ $TP_  ;#Set the Transmissiont Power w.r.t Distance
    }
}


#for VAAD
Agent/Vaad set data_exp_int_ 5
Agent/Vaad set data_con_int_ 0.01
Agent/Vaad set token_con_int_ 0.01
Agent/Vaad set verbose_ 0
Agent/Vaad set drop_debug_ 0
Agent/Vaad set base_pkt_id_ 0
Agent/Vaad set L 300
Agent/Vaad set prob 0
Agent/Vaad set type_ 0
Agent/Vaad set flag_ 0
Agent/Vaad set ID_ 0
Agent/Vaad set off_vaad_ 100
Agent/Vaad set ED 3000
#UC3M ADDED NEXT LINE
Agent/Vaad set longCad 0
# =====================================================================




TestSuite instproc init {} {
	global opt tracefd topo chan prop 
	global node_ god_ gkeeper
	$self instvar ns_ testName_

#remove data packet headers
        remove-packet-header AODV Diffusion IMEP MIP Smac TORA RTP SCTP aSRM CtrMcast mcastCtrl MFTP PGM PGM_SPM PGM_NAK SRM SRMEXT       

	set ns_         [new Simulator]
    if {[string compare $testName_ "dsr"] && \
	    [string compare $testName_ "vaad"] } {
	     $ns_ set-address-format hierarchical
	     AddrParams set domain_num_ 3
	     lappend cluster_num 2 1 1
	     AddrParams set cluster_num_ $cluster_num
	     lappend eilastlevel 1 1 4 1
	     AddrParams set nodes_num_ $eilastlevel
        }  
	set chan	[new $opt(chan)]
	set prop	[new $opt(prop)]
	set topo	[new Topography]
	set tracefd	[open $opt(tr) w]
        
        $ns_ trace-all tracefd ;#trace all

	#set opt(rp) $testName_
	$topo load_flatgrid $opt(x) $opt(y)
	$prop topography $topo
	#
	# Create God
	#
	$self create-god $opt(nn)
	
	#
	# log the mobile nodes movements if desired
	#
	if { $opt(lm) == "on" } {
		$self log-movement
	}
	puts $tracefd "M 0.0 nn:$opt(nn) x:$opt(x) y:$opt(y) rp:$opt(rp)"
	puts $tracefd "M 0.0 sc:$opt(sc) cp:$opt(cp) seed:$opt(seed)"
	puts $tracefd "M 0.0 prop:$opt(prop) ant:$opt(ant)"
}

TestSuite instproc create_gridkeeper { } {

        global gkeeper opt node_
                
        set gkeeper [new GridKeeper]
        
        #initialize the gridkeeper
                
        $gkeeper dimension $opt(x) $opt(y)
 
        #
        # add mobile node into the gridkeeper, must be added after
        # scenario file
        #       

        for {set i 0} {$i < $opt(nn) } {incr i} {
	    $gkeeper addnode $node_($i)
        
	    $node_($i) radius $opt(radius)
        }       
}

Test/vaad instproc init {} {
    global opt node_ god_ ldb_ ragent_ MacTrace
	global dish_ratio_ 
   #uc3m - changed global reaL realED realp 
	global reaL realED realp longCad
     # seed the default RNG
     global defaultRNG
     $defaultRNG seed 0

    set dish_ratio_ 0.0
    $self instvar ns_ testName_
    set testName_       vaad
    set opt(rp)         vaad
    set opt(ragent)     Agent/Vaad
    set opt(cp)		"cbr-80k-2"  
    set opt(sc)         "Urban_Mhnt_750n_1000s_1.tcl" ;#"vaad_10k_nolight_1800.tcl" ;# "Urban_Mhnt_750n_1000s_1.tcl" ;#"vaad_10k_nolight_1800.tcl" ;#"single3.trace";#mobility file.UC3M - REMEMBER TO CHANGE THIS
    set opt(nn)		751 ; #201 ;#751 ;#201 ;#61;#781   ;#781 nodes UC3M - REMEMBER TO CHANGE THIS
    set opt(stop)	950.0 ;#1793 ;#950.0 ;#793 ;#950.0 ;#1793 ;#900.0 UC3M - REMEMBER TO CHANGE THIS

    $self next

    # size_ is a uniform random variable describing packet sizes
    set size_ [new RandomVariable/Uniform]
    $size_ set min_ 0
    $size_ set max_ 139    ;#cannot set 140 or 141 as dishonest node.    
    
    $ns_ set-address-format def ; #commented by Zhengming
    set ldb_ [new LocDbase]
    $ldb_ nnodes $opt(nn)
    for {set i 0} {$i < $opt(nn) } {incr i} {        
	$testName_-create-mobile-node $i
#	if { [Agent/GPSR set use_mac_] && $MacTrace == "OFF" } {
#	    set macdropt [cmu-trace Drop "MAC" $node_($i)]
#	    [$node_($i) set mac_(0)] drop-target $macdropt
#	}
	$ragent_($i) install-tap [$node_($i) set mac_(0)]
	$ldb_ register [$node_($i) address?] $node_($i)
	$ragent_($i) ldb $ldb_
	#set L and prob 
        $ragent_($i) set prob $realp
	#$ragent_($i) set ED $realED
	$ragent_($i) set L $reaL
	#UC3M - NEXT LINE ADDED TO SEND NODES THE CHAIN LENGTH
	$ragent_($i) set longCad $longCad
	
	#set the useful parameters	
	#set the dishonest nodes according to the ratio
    }
    #set the flag of node 200 to be the source node
    $ragent_([expr {$opt(nn)-1}]) set flag_ 1 

    ###################################################33
    #Set the dropping node and misbehaving node accordingly.	
    set mid_size [expr {$opt(nn) * $dish_ratio_}]
    set mis_size [expr {round($mid_size)}]
    ##This will set the nodes to drop data packet.
    set disc_ratio 0.0 
    set mid_size [expr {$opt(nn) * $disc_ratio}]
    set drop_size [expr {round($mid_size)}];
    puts [format "Total Mis number %-4d  drop number %-4d  %f " $mis_size $drop_size $dish_ratio_]
    
    for {set j 0} {$j < $drop_size } {incr j} {  
       #set the dishonest nodes according to the ratio
        $size_ use-rng $defaultRNG
        set index [expr round([$size_ value])]
        $ragent_($index) set flag_ 1
        puts [format "drop index %d " $index]
    }
    for {set j 0} {$j < $mis_size } {incr j} {  
       #set the dishonest nodes according to the ratio
        $size_ use-rng $defaultRNG
        set index [expr round([$size_ value])]
        $ragent_($index) set flag_ 2
        puts [format "Mis index %d " $index]
    }    
    #$ns_ halt
	
#        $ragent_(140) set type_ 1	;#source node
#	$ragent_(141) set type_ 2	;#destination node
	
    puts "Loading connection pattern..."
    source $opt(cp)
    
    puts "Loading scenario file..."
    source $opt(sc)
    puts "Load complete..."

    #
    # Tell all the nodes when the simulation ends
    #
    for {set i 0} {$i < $opt(nn) } {incr i} {
	$ns_ at $opt(stop).000000001 "$node_($i) reset";
    }
    
    $ns_ at $opt(stop).000000001 "puts \"NS EXITING...\" ;" 
    #$ns_ halt"
    $ns_ at $opt(stop).1 "$self finish"
}

Test/vaad instproc run {} {
    $self instvar ns_
    puts "Starting Simulation..."
    $ns_ run
}


Test/dsr instproc init {} {
    global opt node_ god_
    $self instvar ns_ testName_
    set testName_       dsr
    set opt(rp)         dsr
#    set opt(cp)         "../mobility/scene/cbr-50-20-4-512"
#    set opt(sc)         "../mobility/scene/scen-670x670-50-600-20-0" ;
    set opt(nn)         50
    set opt(stop)       900.0
    set opt(cp)		"cbr-50-2Kbps"

    $self next

    for {set i 0} {$i < $opt(nn) } {incr i} {
        $testName_-create-mobile-node $i
    }
    puts "Loading connection pattern..."
    source $opt(cp)

    puts "Loading scenario file..."
    source $opt(sc)
    puts "Load complete..."

    #
    # Tell all the nodes when the simulation ends
    #
    for {set i 0} {$i < $opt(nn) } {incr i} {
        $ns_ at $opt(stop).000000001 "$node_($i) reset";
    }

    $ns_ at $opt(stop).000000001 "puts \"NS EXITING...\" ;"
    #$ns_ halt"
    $ns_ at $opt(stop).1 "$self finish"
}

Test/dsr instproc run {} {
    $self instvar ns_
    puts "Starting Simulation$self instvar node_..."
    $ns_ run
}

#=================================================
# UC3M - COUNT AMOUNT OF COUPONS NOT TAKEN
#=================================================
proc countCouponsNotTaken {} {        ;# (called from the C++ part)
   global couponNotTaken
   incr couponNotTaken
  
}

#=================================================
# UC3M - COUNT AMOUNT OF COUPONS TAKEN
#=================================================
proc countCouponsTaken {} {        ;# (called from the C++ part)
   #puts "FORWARDING ADDED SUCCESSFULLY!" 
   global couponTaken
   incr couponTaken
  
}

#=================================================
# UC3M - COUNT AMOUNT OF COUPONS SENT
#=================================================
proc countCouponsSent {} {        ;# (called from the C++ part)
   global couponSent
   incr couponSent
  
}

#=================================================
# UC3M - COUNT COMMERCIAL FEEDBACK MAX DISTANCE
#=================================================
proc countFeedbackDistance {} {        ;# (called from the C++ part)
   global commFeedbackDistance
   incr commFeedbackDistance
  
}

#=================================================
# UC3M - COUNT COMMERCIAL FEEDBACK NO NEIGHBOURS
#=================================================
proc countFeedbackNoNeighbours {} {        ;# (called from the C++ part)
   global commFeedbackNoNeighbours
   incr commFeedbackNoNeighbours
  
}

#=================================================
# UC3M - COUNT FINISHED CHAINS
#=================================================
proc countFinishedChain {} {        ;# (called from the C++ part)
   global commFeedbackFinishedChain
   incr commFeedbackFinishedChain
  
}

proc cmu-trace { ttype atype node } {
	global ns tracefd
    
        set ns [Simulator instance]
	if { $tracefd == "" } {
		return ""
	}
	set T [new CMUTrace/$ttype $atype]
	$T target [$ns set nullAgent_]
	$T attach $tracefd
        $T set src_ [$node id]

        $T node $node

	return $T
}

TestSuite instproc finish {} {
	$self instvar ns_
	global quiet
	global couponTaken couponNotTaken couponSent commFeedbackFinishedChain commFeedbackDistance commFeedbackNoNeighbours realp reaL longCad
	#UC3M
        set filename "results.txt"
        # open the filename for writing
        set fileId [open $filename "a+"]
        # send the data to the file -
	puts $fileId "prob $realp L $reaL lenght chain $longCad"
        puts $fileId "--------------------------"
        puts $fileId "TOTAL COUPON CHAINS SENT: $couponSent"
  	puts $fileId "NOT-LAST COUPONS TAKEN: $couponTaken"
	puts $fileId "LAST COUPON TAKEN AND COMMERCIAL FEEDBACK (end of coupon chain): $commFeedbackFinishedChain"
	puts $fileId "COUPONS NOT TAKEN: $couponNotTaken"
	puts $fileId "COMMERCIAL FEEDBACK (max distance): $commFeedbackDistance"
	puts $fileId "COMMERCIAL FEEDBACK (no neighbours): $commFeedbackNoNeighbours"
	puts $fileId "--------------------------"
        # close the file, ensuring the data is written out before you continue
        #  with processing.
        close $fileId

	$ns_ flush-trace
        #if { !$quiet } {
        #        puts "running nam..."
        #        exec nam temp.rands.nam &
        #}
	puts "finishing.."

	exit 0
}

TestSuite instproc create-god { nodes } {
	global tracefd god_
	$self instvar ns_

	set god_ [new God]
	$god_ num_nodes $nodes
}

TestSuite instproc log-movement {} {
	global ns
	$self instvar logtimer_ ns_

	set ns $ns_
	source ../mobility/timer.tcl
	Class LogTimer -superclass Timer
	LogTimer instproc timeout {} {
		global opt node_;
		for {set i 0} {$i < $opt(nn)} {incr i} {
			$node_($i) log-movement
		}
		$self sched 0.1
	}

	set logtimer_ [new LogTimer]
	$logtimer_ sched 0.1
}

TestSuite instproc create-tcp-traffic {id src dst start} {
    $self instvar ns_
    set tcp_($id) [new Agent/TCP]
    $tcp_($id) set class_ 2
    set sink_($id) [new Agent/TCPSink]
    $ns_ attach-agent $src $tcp_($id)
    $ns_ attach-agent $dst $sink_($id)
    $ns_ connect $tcp_($id) $sink_($id)
    set ftp_($id) [new Application/FTP]
    $ftp_($id) attach-agent $tcp_($id)
    $ns_ at $start "$ftp_($id) start"    
}


TestSuite instproc create-udp-traffic {id src dst start} {
    $self instvar ns_
    set udp_($id) [new Agent/UDP]
    $ns_ attach-agent $src $udp_($id)
    set null_($id) [new Agent/Null]
    $ns_ attach-agent $dst $null_($id)
    set cbr_($id) [new Application/Traffic/CBR]
    $cbr_($id) set packetSize_ 512
    $cbr_($id) set interval_ 4.0
    $cbr_($id) set random_ 1
    $cbr_($id) set maxpkts_ 10000
    $cbr_($id) attach-agent $udp_($id)
    $ns_ connect $udp_($id) $null_($id)
    $ns_ at $start "$cbr_($id) start"

}


proc create-vaad-routing-agent { node id } {
    global ns_ ragent_ tracefd opt

    #
    #  Create the Routing Agent and attach it to port 255.
    #
    #set ragent_($id) [new $opt(ragent) $id]
    set ragent_($id) [new $opt(ragent)]
    set ragent $ragent_($id)
    
    #set the node id to the routing agent
    $ragent set ID_ $id

    ## setup address (supports hier-addr) for dsdv agent 
    ## and mobilenode
    set addr [$node node-addr]
    
    $ragent node $node
    $ragent set-self $ragent
    if [Simulator set mobile_ip_] {
	$ragent port-dmux [$node set dmux_]
    }
    $node addr $addr
    $node set ragent_ $ragent
    
    $node attach $ragent 255

    ##$ragent set target_ [$node set ifq_(0)]	;# ifq between LL and MAC
        
    # XXX FIX ME XXX
    # Where's the DSR stuff?
    #$ragent ll-queue inner_proc_[$node get-queue 0]    ;# ugly filter-queue hack
    $ns_ at 0.0 "$ragent_($id) start-vaad"	;# start updates

    #
    # Drop Target (always on regardless of other tracing)
    #
    set drpT [cmu-trace Drop "RTR" $node]
    $ragent drop-target $drpT
    
    #
    # Log Target
    #
    set T [new Trace/Generic]
    $T target [$ns_ set nullAgent_]
    $T attach $tracefd
    $T set src_ $id
    $ragent tracetarget $T

    # ifq
    $ragent add-ifq [$node set ifq_(0)]
}


#This procedure will create a mobile node and attach the vaad routing agent to it.
#watch it.
proc vaad-create-mobile-node { id args } {
    global ns ns_ chan prop topo tracefd opt node_
    global inner_proc_ outer_proc_ fec_proc_
    
    set inner_proc_ ""
    set outer_proc_ ""
    set fec_proc_ ""
    
    set ns_ [Simulator instance]
    #if {[Simulator set EnableHierRt_]} {
	#if [Simulator set mobile_ip_] {
	 #   set node_($id) [new MobileNode/MIPMH $args]
	#} else {
	 #   set node_($id) [new Node/MobileNode/BaseStationNode $args]
	#}
#    } else {
	set node_($id) [new Node/MobileNode]
 #   }
    set node $node_($id)
    $node random-motion 0		;# disable random motion
    $node topography $topo
    
    #
    # This Trace Target is used to log changes in direction
    # and velocity for the mobile node.
    #
    set T [new Trace/Generic]
    $T target [$ns_ set nullAgent_]
    $T attach $tracefd
    $T set src_ $id
    $node log-target $T
    
    $node add-interface $chan $prop $opt(ll) $opt(mac)	\
	    $opt(ifq) $opt(ifqlen) $opt(netif) $opt(ant) $topo $inner_proc_ $outer_proc_ $fec_proc_

    #
    # Create a Routing Agent for the Node
    #
    create-$opt(rp)-routing-agent $node $id
    
    # ============================================================
    
	if { $opt(pos) == "Box" } {
		#
		# Box Configuration
		#
		set spacing 200
		set maxrow 7
		set col [expr ($id - 1) % $maxrow]
		set row [expr ($id - 1) / $maxrow]
		$node set X_ [expr $col * $spacing]
		$node set Y_ [expr $row * $spacing]
		$node set Z_ 0.0
		$node set speed_ 0.0

		$ns_ at 0.0 "$node_($id) start"
	}
	return $node
}

proc runtest {arg} {
	global quiet opt reaL realp longCad	
	set quiet 0
	set reaL 300   	
        set realp 0.00
	#UC3M - ADDED LONG CHAIN
	set longCad 0
	set b [llength $arg]
	#UC3M - NEW PARAMETER ADDED, THUS CHANGING	if $b == 5  
        if {$b == 6} {
	    set test [lindex $arg 0]
	    set opt(tr) [lindex $arg 1]
	    #set opt(sc) [lindex $arg 2]
	    set realp [lindex $arg 2] 
            set reaL [lindex $arg 3]
            puts "set reaL and realED!"
	    #UC3M	    set opt(mac) [lindex $arg 4]
      	    set opt(mac) [lindex $arg 5]
            set longCad [lindex $arg 4]
	} elseif {$b == 2} {
	    set test [lindex $arg 0]	          		
	} else {
	    usage
	}
	set t [new Test/$test]
	$t run
}

global argv arg0
default_options
runtest $argv
#runtest {vaad mobile-n.tr 3000 3000 Mac/802_11}
