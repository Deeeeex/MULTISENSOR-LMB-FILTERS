function checkTcAdapter()
setupTcDependency();
ospa=struct('winlen_lm',5,'min_track_len',2,'use_consecutive_len',false, ...
    'c_lm',12,'p',1,'metric_type','ospa_union','unique_multiplier',1e5);
N=3; T=6; agents=cell(1,N); memories=cell(1,N);
for n=1:N
    est=struct('source_id',n,'X',{cell(T,1)},'L',{cell(T,1)},'N',zeros(T,1));
    agents{n}=struct('source_info',struct('neighbor_id',setdiff(1:N,n)), ...
        'est',est,'est_fused',est,'model',struct('ospa',ospa), ...
        'l_space',{cell(N,1)},'l_asso_hist',{cell(N,N)});
    memories{n}=struct('space',{cell(N,1)},'association',{cell(N,N)});
end
comparisons=0;
for t=1:T
    decoded=cell(1,N);
    for n=1:N
        % Independent node labels, one shared kinematic target; source 3 has
        % a distant exclusive target and a final-frame unconfirmed track.
        x=[t+.1*n;1;.2*n;0]; l=[1;n*1e5+1];
        if n==3, x=[x,[50+t;1;0;0]]; l=[l,[1;n*1e5+2]]; end
        if n==3 && t==T, x=[x,[90;0;0;0]]; l=[l,[t;n*1e5+3]]; end
        agents{n}.est.X{t}=x; agents{n}.est.L{t}=l; agents{n}.est.N(t)=size(l,2);
        decoded{n}=tcHistoryPacket(agents{n}.est,t,5);
    end
    native=fusion_main_tc(struct('ospa',ospa),agents,t);
    for n=1:N
        sources=[n,setdiff(1:N,n)];
        [x,~,memories{n}]=tcFuseReceiver(decoded(sources),sources,memories{n},ospa,t);
        assert(isequal(x,native{n}.est_fused.X{t}),'Adapter differs from author kinematic output.');
        assert(~any(x(1,:)==90),'Fresh unmatched track must not pass the native length filter.');
        comparisons=comparisons+1;
    end
    agents=native;
end
% Packet omission and isolation cannot access another node's current output.
[x,~,~]=tcFuseReceiver(decoded(1),1,memories{1},ospa,T);
assert(isequal(x,agents{1}.est.X{T}));
fprintf('TC ADAPTER PASS: %d exact native multi-node comparisons; packet round trips, isolation, independent labels and unmatched-history rule.\n',comparisons);
end
