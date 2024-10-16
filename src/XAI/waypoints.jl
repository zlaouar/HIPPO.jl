function best_a_node_visits(tree::BasicPOMCP.POMCPTree,index::Int)
    root_actions = tree.children[index]
    max_node = 0
    max_visits = 0
    for i in root_actions
        vis = tree.n[i]
        if vis > max_visits
            max_node = i
            max_visits = vis
        end
    end
    return max_node, max_visits
end

function best_a_node_value(tree::BasicPOMCP.POMCPTree,index::Int)
    root_actions = tree.children[index]
    max_node = 0
    max_visits = -Inf
    for i in root_actions
        vis = tree.v[i]
        if vis > max_visits
            max_node = i
            max_visits = vis
        end
    end
    return max_node, max_visits
end

function best_a_nodes_visits(tree::BasicPOMCP.POMCPTree,index::Int,n::Int)
    root_actions = tree.children[index]
    idxs = partialsortperm(tree.n[root_actions], 1:n, rev=true)
    return root_actions[idxs], tree.n[root_actions][idxs]
end

function best_a_nodes_value(tree::BasicPOMCP.POMCPTree,index::Int,n::Int)
    root_actions = tree.children[index]
    idxs = partialsortperm(tree.v[root_actions], 1:n, rev=true)
    return root_actions[idxs], tree.n[root_actions][idxs]
end

function recursive_children(ind,s,tree,pomdp,state_list,opac_list,parent_list,parent,total_vis,depth,d,prev_prob,n_actions,from_node::Bool)
    if !isnothing(ind) && d <= depth 
        visits = tree.total_n[ind]
        if visits != 0
            if from_node && d==2
                ais_new,_ = best_a_nodes_value(tree,ind,1)
                total_vis = tree.n[ais_new[1]]
            else
                ais_new,_ = best_a_nodes_value(tree,ind,n_actions)
            end

            for ai_new in ais_new
                # if (d == 2) || (d == 3)
                #     @info "wypt current: $(tree.a_labels[ai_new])"
                #     @info "wypt max_ind: $ai_new"
                #     @info tree.children[ind]
                # end
                if !isterminal(pomdp, s)
                    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
                    spro = HIPPO.bounce(pomdp, s.robot, HIPPO.actiondir[tree.a_labels[ai_new]])
                    sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
                    if !isterminal(pomdp, sp)
                        sp_idx = findall(x->x==sp,state_list)
                        prob = tree.n[ai_new]/total_vis #prev_prob*(visits/total_vis)
                        if isempty(sp_idx)
                            push!(state_list,sp)
                            push!(opac_list,prob)
                            push!(parent_list,parent)
                        else
                            s_idx = findall(x->parent_list[x]==s,sp_idx)
                            if isempty(s_idx)
                                push!(state_list,sp)
                                push!(opac_list,prob)
                                push!(parent_list,parent)
                            else
                                opac_list[s_idx[1]] += prob
                            end
                        end

                        new_inds = Int[]
                        for o in POMDPs.observations(pomdp)
                            idx = get(tree.o_lookup,(ai_new,o),nothing)
                            !isnothing(idx) && push!(new_inds,idx)
                        end
                        # total_vis = max(sum(tree.total_n[new_inds]),1)
                        for ind2 in new_inds
                            recursive_children(ind2,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,d+1,prob,n_actions,from_node)
                        end
                    end
                end
            end
        end
    end
end

function get_children(pomdp::PachPOMDP,b,tree::BasicPOMCP.POMCPTree;depth=2,n_actions=4)
    state_list = S[]
    opac_list = Float64[]
    parent_list = S[]
    
    ai_new,_= best_a_nodes_value(tree,1,1)
    ai_new = ai_new[1]
    root_n = tree.n[ai_new]
    s = first(support(b))
    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
    spro = HIPPO.bounce(pomdp, s.robot, HIPPO.actiondir[tree.a_labels[ai_new]])
    sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
    push!(state_list,sp)
    push!(opac_list,tree.n[ai_new]/root_n)
    push!(parent_list,s)

    new_inds = Int[]
    for o in POMDPs.observations(pomdp)
        idx = get(tree.o_lookup,(ai_new,o),nothing)
        !isnothing(idx) && push!(new_inds,idx)
    end
    # [get(tree.o_lookup,(ai_new,o),nothing) for o in POMDPs.observations(pomdp)]
    total_vis = max(sum(tree.total_n[new_inds]),1)

    for ind in new_inds
        recursive_children(ind,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,2,1.0,n_actions,false)
    end

    opac_thresh = 0.0
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end
    opac_list = (opac_list).^0.5
    return (state_list,opac_list,parent_list)
end

function get_children(pomdp::TargetSearchPOMDP,b,s,tree::BasicPOMCP.POMCPTree;depth=2,n_actions=4)
    S = statetype(pomdp)
    state_list = S[]
    opac_list = Float64[]
    parent_list = S[]
    
    ai_new,_= best_a_nodes_value(tree,1,1)
    ai_new = ai_new[1]
    root_n = tree.n[ai_new]
    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
    spro = HIPPO.bounce(pomdp, s.robot, HIPPO.actiondir[tree.a_labels[ai_new]])
    sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
    push!(state_list,sp)
    push!(opac_list,tree.n[ai_new]/root_n)
    push!(parent_list,s)

    new_inds = Int[]
    for o in POMDPs.observations(pomdp)
        idx = get(tree.o_lookup,(ai_new,o),nothing)
        !isnothing(idx) && push!(new_inds,idx)
    end
    # [get(tree.o_lookup,(ai_new,o),nothing) for o in POMDPs.observations(pomdp)]
    total_vis = max(sum(tree.total_n[new_inds]),1)

    for ind in new_inds
        recursive_children(ind,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,2,1.0,n_actions,false)
    end

    opac_thresh = 0.0
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end
    opac_list = (opac_list).^0.5
    return (state_list,opac_list,parent_list)
end

function get_children_from_node(pomdp::PachPOMDP,b,tree::BasicPOMCP.POMCPTree,obs,aprev;depth=2,n_actions=4)
    #Borrow from next_action?
    S = statetype(pomdp)
    state_list = S[]
    opac_list = Float64[]
    parent_list = S[]

    # ai_new,_= best_a_node(tree,1)
    # @info tree.a_labels[ai_new]
    # root_n = tree.n[ai_new]
    # s = first(support(b))
    # sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
    # push!(state_list,sp)
    # push!(opac_list,tree.n[ai_new]/root_n)
    # push!(parent_list,s)

    #From next_action
    # ha = findfirst(a -> a == aprev, tree.a_labels[t.children[1]])
    # hao = get(t.o_lookup, (ha, obs), 0)
    # if hao == 0
    #     @warn "wp reached observation not in tree"
    #     inchrome(D3Tree(t))
    #     return :stay
    # end
    # @info "wypt obs: $obs"
    ai_new = findfirst(a -> a == aprev, tree.a_labels[tree.children[1]])
    # @info ai_new
    # @info "wypt prev act: $ai_new"
    sp = first(support(b))
    
    idx = get(tree.o_lookup,(ai_new,obs),nothing)
    # @info "wypt prev: $idx"
    # @info idx
    # [get(tree.o_lookup,(ai_new,o),nothing) for o in POMDPs.observations(pomdp)]
    if !isnothing(idx) 
        total_vis = tree.total_n[idx]
        recursive_children(idx,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth+1,2,1.0,n_actions,true)
    end

    opac_thresh = 0.0
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end
    opac_list = (opac_list).^0.5
    return (state_list,opac_list,parent_list)
end

function recursive_children(ind,s,tree,pomdp::UnifiedPOMDP,state_list,opac_list,parent_list,parent,total_vis,depth,d,prev_prob,n_actions,from_node::Bool)
    if !isnothing(ind) && d <= depth 
        visits = tree.total_n[ind]
        if visits != 0
            if from_node && d==2
                ais_new,_ = best_a_nodes_value(tree,ind,1)
                total_vis = tree.n[ais_new[1]]
            else
                ais_new,_ = best_a_nodes_value(tree,ind,n_actions)
            end

            for ai_new in ais_new
                # if (d == 2) || (d == 3)
                #     @info "wypt current: $(tree.a_labels[ai_new])"
                #     @info "wypt max_ind: $ai_new"
                #     @info tree.children[ind]
                # end
                if !isterminal(pomdp, s)
                    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
                    # spro = HIPPO.bounce(pomdp, s.robot, HIPPO.actiondir[tree.a_labels[ai_new]])
                    # sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
                    if !isterminal(pomdp, sp)
                        sp_idx = findall(x->x==sp,state_list)
                        prob = tree.n[ai_new]/total_vis #prev_prob*(visits/total_vis)
                        if isempty(sp_idx)
                            push!(state_list,sp)
                            push!(opac_list,prob)
                            push!(parent_list,parent)
                        else
                            s_idx = findall(x->parent_list[x]==s,sp_idx)
                            if isempty(s_idx)
                                push!(state_list,sp)
                                push!(opac_list,prob)
                                push!(parent_list,parent)
                            else
                                opac_list[s_idx[1]] += prob
                            end
                        end

                        new_inds = Int[]
                        for o in POMDPs.observations(pomdp)
                            idx = get(tree.o_lookup,(ai_new,o),nothing)
                            !isnothing(idx) && push!(new_inds,idx)
                        end
                        # total_vis = max(sum(tree.total_n[new_inds]),1)
                        for ind2 in new_inds
                            recursive_children(ind2,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,d+1,prob,n_actions,from_node)
                        end
                    end
                end
            end
        end
    end
end

function get_children(pomdp::UnifiedPOMDP,b,s,tree::BasicPOMCP.POMCPTree;depth=2,n_actions=4)
    S = statetype(pomdp)
    state_list = S[]
    opac_list = Float64[]
    parent_list = S[]
    
    ai_new,_= best_a_nodes_value(tree,1,1)
    ai_new = ai_new[1]
    root_n = tree.n[ai_new]
    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
    # spro = HIPPO.bounce(pomdp, s.robot, HIPPO.actiondir[tree.a_labels[ai_new]])
    # sp = HIPPO.FullState(spro,sp.target,sp.visited,sp.battery)
    push!(state_list,sp)
    push!(opac_list,tree.n[ai_new]/root_n)
    push!(parent_list,s)

    new_inds = Int[]
    for o in POMDPs.observations(pomdp)
        idx = get(tree.o_lookup,(ai_new,o),nothing)
        !isnothing(idx) && push!(new_inds,idx)
    end
    # [get(tree.o_lookup,(ai_new,o),nothing) for o in POMDPs.observations(pomdp)]
    total_vis = max(sum(tree.total_n[new_inds]),1)

    for ind in new_inds
        recursive_children(ind,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,2,1.0,n_actions,false)
    end

    opac_thresh = 0.0
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end
    opac_list = (opac_list).^0.5
    return (state_list,opac_list,parent_list)
end