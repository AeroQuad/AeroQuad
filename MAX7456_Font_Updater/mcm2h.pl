#!/usr/bin/perl

open(MYINPUTFILE, "<font.mcm");
open(MYOUTPUTFILE, ">font.h");
my $count = -1;
print MYOUTPUTFILE "PROGMEM const byte fontdata[16384] = {\n  ";
while(<MYINPUTFILE>)
{
    my($line) = $_;
    chomp($line);
    if ($count!=-1)
    {
	my ($x) = oct("0b".$line);
	printf MYOUTPUTFILE "0x%02x", $x;
        if ($count!=16383) {
	    print MYOUTPUTFILE ",";
	    if (($count%32)==31) {
		print MYOUTPUTFILE "\n  ";
	    }
	}
    }
    $count++;
}
print MYOUTPUTFILE "};\n";

close(MYINPUTFILE);
close(MYOUTPUTFILE);
