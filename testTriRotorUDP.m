function testTriRotorUDP
    u = udpport("datagram","IPV4", ...
        "LocalPort", 5501, ...
        "Timeout",   2.0);

    fprintf("Listening on UDP 5501...\n");

    for k = 1:30
        if isprop(u,"NumDatagramsAvailable")
            n = u.NumDatagramsAvailable;
        elseif isprop(u,"NumBytesAvailable")
            n = u.NumBytesAvailable;
        else
            error("udpport has neither NumDatagramsAvailable nor NumBytesAvailable.");
        end

        if n == 0
            pause(0.05);
            continue;
        end

        pkt = read(u, 1, "string");
        if isempty(pkt)
            fprintf("%2d: <empty>\n", k);
        else
            s = pkt.Data;
            fprintf("%2d: %s\n", k, s);
        end
    end

    clear u;
end
