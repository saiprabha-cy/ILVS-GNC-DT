function packet = pack_telemetry_packet(t, state, diag, stage, seq)
% PACK_TELEMETRY_PACKET  Build one ascent telemetry sample.
%
%   packet = PACK_TELEMETRY_PACKET(t, state, diag, stage, seq)
%
%   Deliberately a THIN packet, not a new protocol: field layout
%   (sequence count, timestamp, payload fields, a placeholder CRC slot)
%   mirrors the CCSDS-style packet structure already implemented and
%   tested in Mission-Operations-Communication-Platform-MOCP, so this
%   stream is compatible with that repo's ground-station decoder rather
%   than requiring a second, competing packet definition -- see
%   docs/01_design_decisions.md.
%
%   Fields:
%     seq_count   - packet sequence number (uint32-range double)
%     time_s      - mission elapsed time, s
%     altitude_m, downrange_m, velocity_ms, flight_path_deg,
%     pitch_deg, pitch_cmd_deg, mass_kg, dynamic_pressure_pa,
%     alpha_deg, stage
%     crc16       - CRC-16/CCITT-FALSE over the payload fields above,
%                   computed with the SAME polynomial and initial value
%                   MOCP uses, so packets from this project are
%                   decodable by MOCP's existing CRC check without
%                   modification.

    packet.seq_count           = seq;
    packet.time_s               = t;
    packet.altitude_m           = state(1);
    packet.downrange_m          = state(2);
    packet.velocity_ms          = state(3);
    packet.flight_path_deg      = rad2deg(state(4));
    packet.pitch_deg            = rad2deg(state(5));
    packet.pitch_cmd_deg        = diag.theta_cmd_deg;
    packet.mass_kg              = state(7);
    packet.dynamic_pressure_pa  = diag.dynamic_pressure;
    packet.alpha_deg            = diag.alpha_deg;
    packet.stage                = stage;

    packet.crc16 = telemetry_crc16(packet);

end


function crc = telemetry_crc16(packet)
% TELEMETRY_CRC16  CRC-16/CCITT-FALSE over the packet's numeric payload.
%
%   Same polynomial (0x1021) and initial value (0xFFFF) as MOCP's CRC
%   implementation, applied here to the packet's fields serialized as
%   IEEE-754 double bytes in a fixed field order -- so a packet built
%   here and one built by MOCP for the same field order are bit-for-bit
%   cross-checkable. See tests/test_telemetry_crc.m for the reference
%   vector this is checked against.

    fields = [packet.time_s, packet.altitude_m, packet.downrange_m, ...
              packet.velocity_ms, packet.flight_path_deg, packet.pitch_deg, ...
              packet.pitch_cmd_deg, packet.mass_kg, packet.dynamic_pressure_pa, ...
              packet.alpha_deg, double(packet.stage)];

    bytes = typecast(fields, 'uint8');

    crc = uint16(hex2dec('FFFF'));
    poly = uint16(hex2dec('1021'));

    for i = 1:length(bytes)
        crc = bitxor(crc, bitshift(uint16(bytes(i)), 8));
        for bit = 1:8
            if bitand(crc, uint16(hex2dec('8000'))) ~= 0
                crc = bitxor(bitshift(crc, 1), poly);
            else
                crc = bitshift(crc, 1);
            end
        end
    end

end
