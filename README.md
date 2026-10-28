# udpbbr: UDP file transfer with a from-scratch BBRv3

Example implementation of a UDP client/server pair utilizing BBRv3 congestion algos
to error-correct datagram drops.

I created a 32-bit version as well. Client runs on my legacy 2.0.35 kernel, 486DX-33 w/10 Mbps network
card while the server runs on my modern 64-bit Linux workstation across my 1Gbps internal network.

Perl directory has a 'flood.pl' script that bombards the lowly 486 network interface with UDP datagrams
to mimic congestion.

Pics or it didn't happen. In this case, how about videos?


