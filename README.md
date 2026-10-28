# udpbbr: UDP file transfer with a from-scratch BBRv3

Example implementation of a UDP client/server pair utilizing BBRv3 congestion algos
to error-correct datagram drops.

I created a 32-bit version as well. Client runs on a my legacy 486DX-33 w/10 Mbps network
card while the server runs on my modern 64-bit Linux workstation.

Perl directory has a 'flood.pl' script that bombards the lowly 486 network interface with UDP datagrams
to mimic congestion.

[Watch the demo video](https://github.com/kodell-64/udp-bbr/demos/demo-i386-client.ogv)


