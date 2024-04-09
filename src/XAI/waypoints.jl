function best_a_node(tree::BasicPOMCP.POMCPTree,index::Int)
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

function recursive_children(ind,s,tree,pomdp,state_list,opac_list,parent_list,parent,total_vis,depth,d,prev_prob)
    if !isnothing(ind) && d <= depth 
        visits = tree.total_n[ind]
        if visits != 0
            ai_new,_ = best_a_node(tree,ind)
            # if d == 2
            #     @info "Wypt current: $(tree.a_labels[ai_new])"
            #     @info ai_new
            # end
            sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
            
            sp_idx = findall(x->x==sp,state_list)
            prob = prev_prob*(visits/total_vis)
            if isempty(sp_idx)
                push!(state_list,sp)
                push!(opac_list,prob)
                push!(parent_list,parent)
            else
                opac_list[sp_idx[1]] += prob
            end

            new_inds = Int[]
            for o in POMDPs.observations(pomdp)
                idx = get(tree.o_lookup,(ai_new,o),nothing)
                !isnothing(idx) && push!(new_inds,idx)
            end
            total_vis = max(sum(tree.total_n[new_inds]),1)
            for ind2 in new_inds
                recursive_children(ind2,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,d+1,prob)
            end
        end
    end
end

function get_children(pomdp::PachPOMDP{S,A,O},b,tree::BasicPOMCP.POMCPTree;depth=2) where {S,A,O}
    state_list = S[]
    opac_list = Float64[]
    parent_list = S[]
    
    ai_new,_= best_a_node(tree,1)
    root_n = tree.n[ai_new]
    s = first(support(b))
    sp = @gen(:sp)(pomdp, s, tree.a_labels[ai_new])
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
        recursive_children(ind,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth,2,1.0)
    end

    opac_thresh = 0.4
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end

    return (state_list,opac_list,parent_list)
end

function get_children_from_node(pomdp::PachPOMDP{S,A,O},b,tree::BasicPOMCP.POMCPTree,obs,aprev;depth=2) where {S,A,O}
    #Borrow from next_action?
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
    # @info "wypt prev: $aprev"
    sp = first(support(b))
    
    idx = get(tree.o_lookup,(ai_new,obs),nothing)
    # @info idx
    # [get(tree.o_lookup,(ai_new,o),nothing) for o in POMDPs.observations(pomdp)]
    if !isnothing(idx) 
        total_vis = tree.total_n[idx]
        recursive_children(idx,sp,tree,pomdp,state_list,opac_list,parent_list,sp,total_vis,depth+1,2,1.0)
    end

    opac_thresh = 0.4
    for i in eachindex(opac_list)
        opac_list[i]<opac_thresh ? opac_list[i]=opac_thresh : nothing
    end

    return (state_list,opac_list,parent_list)
end

