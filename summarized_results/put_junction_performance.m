function tmp = put_junction_performance(tmp, flow, travel_time)
    tmp(1).flow = [tmp(1).flow; flow.J1];
    tmp(2).flow = [tmp(2).flow; flow.J2];
    tmp(3).flow = [tmp(3).flow; flow.J3];
    tmp(4).flow = [tmp(4).flow; flow.J4];
    tmp(5).flow = [tmp(5).flow; flow.J5];
    tmp(6).flow = [tmp(6).flow; flow.J6];
    tmp(7).flow = [tmp(7).flow; flow.J7];
    tmp(8).flow = [tmp(8).flow; flow.J8];
    tmp(9).flow = [tmp(9).flow; flow.J9];
    tmp(10).flow = [tmp(10).flow; flow.J10];
    
    tmp(1).travel_time = [tmp(1).travel_time; travel_time.J1];
    tmp(2).travel_time = [tmp(2).travel_time; travel_time.J2];
    tmp(3).travel_time = [tmp(3).travel_time; travel_time.J3];
    tmp(4).travel_time = [tmp(4).travel_time; travel_time.J4];
    tmp(5).travel_time = [tmp(5).travel_time; travel_time.J5];
    tmp(6).travel_time = [tmp(6).travel_time; travel_time.J6];
    tmp(7).travel_time = [tmp(7).travel_time; travel_time.J7];
    tmp(8).travel_time = [tmp(8).travel_time; travel_time.J8];
    tmp(9).travel_time = [tmp(9).travel_time; travel_time.J9];
    tmp(10).travel_time = [tmp(10).travel_time; travel_time.J10];
end