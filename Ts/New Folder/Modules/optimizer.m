function [opt_t, opt_d, speeds, path, cost, Nodes, Edges] = optimizer( ...
    tf, final_distance, v_min, v_max, b1, b2, traffic_lights )

    [t_min, t_max] = velocity_pruning(traffic_lights, tf, final_distance, v_min, v_max);
    d = [traffic_lights.distance];
    nIntersections = length(traffic_lights);

    % Costruzione nodi (Nodes)
    Nodes = struct('id', {}, 't', {}, 'd', {}, 'int', {});
    nodeId = 1;
    Nodes(nodeId) = struct('id', nodeId, 't', 0, 'd', 0, 'int', 0);
    nodeId = nodeId + 1;

    for i = 1:nIntersections
        light = traffic_lights(i);
        for k = 0 : ceil(tf / light.cycle_time)
            cycle_start = k*light.cycle_time + light.offset;
            if light.green_start <= light.green_end
                abs_green_start = cycle_start + light.green_start;
                abs_green_end   = cycle_start + light.green_end;
                if abs_green_start <= tf
                    overlap_start = max(abs_green_start, t_min(i));
                    overlap_end   = min(abs_green_end,   t_max(i));
                    if overlap_start < overlap_end
                        middle_time = ceil((overlap_start + overlap_end) / 2);
                        Nodes(nodeId) = struct('id', nodeId, 't', middle_time, ...
                                               'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            else
                abs_green_start_1 = cycle_start + light.green_start;
                abs_green_end_1   = cycle_start + light.cycle_time;
                if abs_green_start_1 <= tf
                    ov_start = max(abs_green_start_1, t_min(i));
                    ov_end   = min(abs_green_end_1,   t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start + ov_end) / 2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, ...
                                               'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
                abs_green_start_2 = cycle_start;
                abs_green_end_2   = cycle_start + light.green_end;
                if abs_green_end_2 <= tf
                    ov_start = max(abs_green_start_2, t_min(i));
                    ov_end   = min(abs_green_end_2,   t_max(i));
                    if ov_start < ov_end
                        mid_t = (ov_start + ov_end) / 2;
                        Nodes(nodeId) = struct('id', nodeId, 't', mid_t, ...
                                               'd', d(i), 'int', i);
                        nodeId = nodeId + 1;
                    end
                end
            end
        end
    end

    Nodes(nodeId) = struct('id', nodeId, 't', tf, 'd', final_distance, ...
                           'int', nIntersections + 1);
    nNodes = nodeId;

    % Costruzione archi (Edges)
    Edges = struct('from', {}, 'to', {}, 'w', {});
    edgeCount = 1;
    for i = 1:nNodes
        lvlA = Nodes(i).int;
        for j = 1:nNodes
            lvlB = Nodes(j).int;
            if lvlB == lvlA + 1
                if Nodes(j).t > Nodes(i).t && Nodes(j).d > Nodes(i).d
                    if Nodes(j).int > 0 && Nodes(j).int <= nIntersections
                        if ~is_green(traffic_lights(Nodes(j).int), Nodes(j).t)
                            continue;
                        end
                    end
                    delta_t = Nodes(j).t - Nodes(i).t;
                    delta_d = Nodes(j).d - Nodes(i).d;
                    v_link = delta_d / delta_t;
                    if v_link >= v_min && v_link <= v_max
                        E_link = delta_t * (b1 * v_link + b2 * v_link^2);
                        Edges(edgeCount) = struct('from', Nodes(i).id, ...
                                                  'to',   Nodes(j).id, ...
                                                  'w',    E_link);
                        edgeCount = edgeCount + 1;
                    end
                end
            end
        end
    end

    % Dijkstra
    [path, cost] = dijkstra(Nodes, Edges, 1, nNodes);

    % Risultati
    opt_nodes = Nodes(path);
    opt_t = arrayfun(@(n)n.t, opt_nodes);
    opt_d = arrayfun(@(n)n.d, opt_nodes);

    speeds = zeros(1, length(path) - 1);
    for k = 1:(length(path) - 1)
        d_ = opt_nodes(k + 1).d - opt_nodes(k).d;
        t_ = opt_nodes(k + 1).t - opt_nodes(k).t;
        speeds(k) = d_ / t_;
    end
end