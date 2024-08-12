function mapaction(p::UnifiedPOMDP, belief::ParticleCollection)
    # max a posteriori action
    return statedir(belief.particles[1].robot, mode(belief).target)
end