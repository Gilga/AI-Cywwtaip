using Pkg
pkg"activate ." # activate Project

# add current path
cd(@__DIR__)
push!(LOAD_PATH,@__DIR__)

try
    @eval using Revise
catch ex
    @warn "Could not load Revise: $ex"
end

while true
  try
    println("Start Script...")
    @eval using Client
    Client.main(ARGS)
  catch ex
    println("Error: script fails on run: $ex")
    open("../error.log","w+") do f Base.showerror(f, ex, catch_backtrace()) end
  end
  print("\nPress enter to restart...")
  input = readline()
  println("Reload Script...")
  try
    Revise.revise()
  catch ex
    open("../error_revise.log","w+") do f Base.showerror(f, ex, catch_backtrace()) end
  end
  println("\33[2J") # clear console: https://rosettacode.org/wiki/Terminal_control/Clear_the_screen#Julia
end
