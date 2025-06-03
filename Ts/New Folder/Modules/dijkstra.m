function [path, cost] = dijkstra(Nodes, Edges, source, target)
    nNodes = length(Nodes);
    cost = inf(1,nNodes);
    prev = nan(1,nNodes);
    cost(source) = 0;
    Q = 1:nNodes;
    while ~isempty(Q)
        [~, idx] = min(cost(Q));
        u = Q(idx);
        Q(Q==u) = [];
        if u == target, break; end
        for e = Edges
            if e.from == u
                v = e.to;
                alt = cost(u) + e.w;
                if alt < cost(v)
                    cost(v) = alt; 
                    prev(v) = u;
                end
            end
        end
    end
    path = [];
    if ~isnan(prev(target)) || target==source
        u = target;
        while ~isnan(u)
            path = [u, path];
            u = prev(u);
        end
    end
end