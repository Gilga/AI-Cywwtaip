@info "Adding Packges..."

# required
using Pkg

# activate this project
pkg"activate ."

# helper
function install_pkgs(pkgs;extern=true)
  installed = false
  pkg_list = Pkg.installed()

  for (pkg_name,use) in pkgs
    pkg_symbol = Symbol(pkg_name)

    if extern && !haskey(pkg_list,pkg_name)
      @debug "Install $pkg_name..."
      @eval Pkg.add($pkg_name)
      installed = true
    end

    if use @eval using $pkg_symbol end

    @debug "$pkg_symbol is ready"
    #exit(0)# exit, cannot create documentations!
  end
  installed
end

# install packages
@info "Install packages..."
pkgs = (x->(x,false)).(readlines(joinpath(root,"REQUIRE")))
if install_pkgs(pkgs) Pkg.update() end

@info "Installed packages:"
show(stdout, "text/plain", sort(collect(Pkg.installed())))
println("\n---------------------------------------------------")