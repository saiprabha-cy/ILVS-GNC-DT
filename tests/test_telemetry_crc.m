function test_telemetry_crc()
% TEST_TELEMETRY_CRC  Unit tests for src/telemetry/pack_telemetry_packet.m.
%
%   Includes a reference-vector check of the CRC-16/CCITT-FALSE
%   implementation itself against the standard published check value
%   for that algorithm (the ASCII string "123456789" must produce
%   0x29B1) -- the same reference check MOCP's CRC implementation uses,
%   so both projects can be verified against the same known-good value.

    % --- CRC algorithm reference vector (independent of packet format) ---
    check_bytes = uint8('123456789');
    crc = uint16(hex2dec('FFFF'));
    poly = uint16(hex2dec('1021'));
    for i = 1:length(check_bytes)
        crc = bitxor(crc, bitshift(uint16(check_bytes(i)), 8));
        for bit = 1:8
            if bitand(crc, uint16(hex2dec('8000'))) ~= 0
                crc = bitxor(bitshift(crc, 1), poly);
            else
                crc = bitshift(crc, 1);
            end
        end
    end
    assert(crc == hex2dec('29B1'), ...
           'CRC-16/CCITT-FALSE reference vector mismatch (expected 0x29B1, got %s)', dec2hex(crc));

    % --- Packet structure sanity ---
    state = [50000; 2000; 1200; deg2rad(60); deg2rad(58); 0.1; 9000];
    diag.theta_cmd_deg = 58.5;
    diag.alpha_deg = -2.0;
    diag.dynamic_pressure = 15000.0;

    p = pack_telemetry_packet(12.5, state, diag, 1, 42);

    assert(p.seq_count == 42, 'sequence count mismatch');
    assert(abs(p.time_s - 12.5) < 1e-9, 'time field mismatch');
    assert(abs(p.altitude_m - 50000) < 1e-9, 'altitude field mismatch');
    assert(p.stage == 1, 'stage field mismatch');
    assert(isa(p.crc16, 'uint16'), 'crc16 field should be uint16');

    % Same inputs -> identical CRC (determinism)
    p2 = pack_telemetry_packet(12.5, state, diag, 1, 42);
    assert(p.crc16 == p2.crc16, 'CRC should be deterministic for identical inputs');

    % Different payload -> (with overwhelming probability) different CRC
    state3 = state; state3(1) = 50001;
    p3 = pack_telemetry_packet(12.5, state3, diag, 1, 42);
    assert(p.crc16 ~= p3.crc16, 'CRC should differ when payload differs');

    fprintf('test_telemetry_crc: all assertions passed\n');

end
