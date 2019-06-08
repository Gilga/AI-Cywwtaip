using Pkg
pkg"activate ."

@info "Installed packages:"
show(stdout, "text/plain", sort(collect(Pkg.installed())))
println("\n---------------------------------------------------")