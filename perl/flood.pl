#!/usr/bin/perl

use strict;
use warnings;
use IO::Socket::INET;
use Time::HiRes qw(usleep);

my $host = shift || "192.168.1.77";
my $port = shift || 9000;
my $message = join(" ", @ARGV) || "Hello from Perl UDP client";

my $sock = IO::Socket::INET->new(
    PeerAddr => $host,
    PeerPort => $port,
    Proto    => 'udp'
) or die "Cannot create UDP socket: $!\n";
my $sent;
while(1)
{
	$sent = $sock->send($message);
	usleep(500);
}

defined $sent
    or die "UDP send failed: $!\n";

print "Sent $sent bytes to $host:$port\n";

close($sock);
