#!/bin/perl

$R=atan2(1,1)/45.;

($el1, $el2, $step) = @ARGV;
if(! $el1 || $el1 == "max") {$el1 = 38.4234};
if(! $el2) {$el2 = $el1};
if(! $step) {$step = 1};

for(;$el1 <= $el2; $el1 += $step) {
	$elarg = (1.7 + $el1)*$R;
	print $el1,"   ", 1366.2 * sin($elarg) / sqrt(29.585-28.551*
	    cos($elarg)), "\n";
}
