using Rsvg, Cairo
function svgtopng(filename_in, filename_out)
    Rsvg.set_default_dpi(300.0)
    r = Rsvg.handle_new_from_file(filename_in);
    d = Rsvg.handle_get_dimensions(r);
    cs = Cairo.CairoImageSurface(d.width,d.height,Cairo.FORMAT_ARGB32);
    c = Cairo.CairoContext(cs);
    Rsvg.handle_render_cairo(c,r);
    Cairo.write_to_png(cs,filename_out);
end

files = readdir(joinpath(@__DIR__, "img"))
for (i,name) in enumerate(files)
    imgstr = "i"
    imgstr = lpad(imgstr, 2, 0) * ".png"
    svgtopng(name, joinpath(@__DIR__,"png/$imgstr"))
end

pngfiles = readdir(joinpath(@__DIR__, "png"))

anim = @animate for f ∈ pngfiles
    img = load(joinpath(@__DIR__, "png", f))
    @show f
    plot(img, grid=false, axis=([], false))
end

gif(anim, joinpath(@__DIR__,"animgif.gif"), fps=2)