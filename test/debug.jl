function updatestep(m, up, s, b, a, o)
    sp = rand(transition(m, s, a))
    bp = update(up, b, a, o)
    return sp, bp
end 