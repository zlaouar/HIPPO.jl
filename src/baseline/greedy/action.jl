function action(p::UnifiedPOMDP, s)
    return statedir(s.robot, mat_to_inertial_inds(p.size, Tuple(argmax(p.reward))))
end