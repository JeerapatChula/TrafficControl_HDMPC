

A1.in = 0;
A1.out = 0;
A1.l1 = get_veh_passed(8, 3) + get_veh_passed(10, 15);
A1.l2 = get_veh_passed(8, 4) + get_veh_passed(10, 16);
B = [1 2 3 4 5 6 14];

for b = 1:length(B)
   load(['boundaries/B', int2str(B(b)), '.mat']);
   A1.in = A1.in + sum(departured_log);
   A1.out = A1.out + sum(arrived_log);
end


A2.in = 0;
A2.out = 0;
A2.l1 = get_veh_passed(1, 3) + get_veh_passed(3, 15);
A2.l2 = get_veh_passed(1, 4) + get_veh_passed(3, 16);
A2.l3 = get_veh_passed(1, 5) + get_veh_passed(2, 11);
A2.l4 = get_veh_passed(1, 6) + get_veh_passed(2, 12);
B = [7 8 9];

for b = 1:length(B)
   load(['boundaries/B', int2str(B(b)), '.mat']);
   A2.in = A2.in + sum(departured_log);
   A2.out = A2.out + sum(arrived_log);
end



A3.in = 0;
A3.out = 0;
A3.l3 = get_veh_passed(4, 5) + get_veh_passed(5, 11);
A3.l4 = get_veh_passed(4, 6) + get_veh_passed(5, 12);
B = [10 11 12 13];

for b = 1:length(B)
   load(['boundaries/B', int2str(B(b)), '.mat']);
   A3.in = A3.in + sum(departured_log);
   A3.out = A3.out + sum(arrived_log);
end



function a = get_veh_passed(J,R)
    load(['intersections_J', int2str(J),'/R', int2str(R), '.mat']);
    a = sum(num_vehicles_in);
end