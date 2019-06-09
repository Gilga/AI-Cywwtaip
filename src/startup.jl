using Pkg
pkg"activate ." # activate Project

using Distributed
AI_THREADS = haskey(ENV, "AI_THREADS") ? parse(Int64, ENV["AI_THREADS"]) : 1
if AI_THREADS > 1 addprocs(AI_THREADS) end # total: procs + main proc
@info "Start $(nprocs()) processes"

@everywhere begin
  using Distributed
  println("Start process $(myid())")
  proc_reloader() = 1 #first(procs())
  proc_messenger() = nprocs() #last(procs())

  if AI_THREADS > 1
    if myid() != proc_messenger()
      pushMsg!(msg::String) = remote_do(pushMsg!, proc_messenger(), "P"*string(myid())*": "*msg)
    else
      global MessageChannel = Channel{String}(Inf)
      pushMsg!(msg::String) = put!(MessageChannel, msg)
    end
  else
    pushMsg! = Base.println
  end

  # add current path
  cd(@__DIR__)
  push!(LOAD_PATH,@__DIR__)
end #@everywhere

try
    @eval using Revise
    #Revise.async_steal_repl_backend() # configure Revise to run revise() before every REPL eval
catch ex
    @warn "Could not load Revise: $ex"
end

# Revise: logger
# Revise.debug_logger()
# using Base.CoreLogging: Debug
#
# function reviseWriteLog()
#   rlogger = Revise.debug_logger()
#   logs = filter(r->r.level==Debug, rlogger.logs)
#   open(joinpath(@__DIR__,"revise.logs"), "w") do io
#       for log in logs
#           println(io, log)
#       end
#   end
# end

# Revise: reload Code
ReviseChannel = Channel(0)
function revise()
  exception = nothing
  try
    Revise.revise()
  catch ex
    exception=ex
  end
  if AI_THREADS > 1 put!(ReviseChannel, exception) end
end

function waitForRevise()
  ex = take!(ReviseChannel)
  if ex != nothing
    @warn "Error on Code reloading: $ex"
  end
end

@everywhere begin
  using Images

  procResult = false
  setProcResult(result) = global procResult = result
  getProcResult() = procResult
  procFinish() = nothing

end #@everywhere

RestartCounter = Int[] #RemoteChannel(()->Channel{Int}(nprocs()))

resetCounter() = global RestartCounter = Int[]
procFinish(id) = push!(RestartCounter, id)
getProcResult() = length(RestartCounter) >= nprocs()-1
sendProcResult(value) = for p in workers() if p != proc_messenger() remote_do(setProcResult, p, value) end end

@everywhere function waitForRestart()
  if AI_THREADS > 1
    remote_do(procFinish, 1, myid())
    while !getProcResult() sleep(1) end #wait until counter reached max

    if myid() != proc_reloader()
      while getProcResult() sleep(1) end #wait until counter resets
    else
      sleep(0.1)
      sendProcResult(true)
      print("\nPress enter to restart...")
      input = readline()
    end
  else
    print("\nPress enter to restart...")
    input = readline()
  end
end

@everywhere begin
  if AI_THREADS > 1 && myid() == proc_messenger()
    #Messages = String[]
    #@async while true
    #  push!(Messages, take!(MessageChannel))
    #end
    println("Read Messages: ")
    while true
      println(take!(MessageChannel))
    end
  else
    pushfirst!(ARGS, string(myid()))
    while true
      try
        pushMsg!("Start Script...")
        @eval using Client
        Client.main(ARGS ;output=pushMsg!)
      catch ex
        pushMsg!("Error: script fails on run: $ex")
        open("../errors[$(myid())].log","w+") do f Base.showerror(f, ex, catch_backtrace()) end
      end
      waitForRestart()
      if AI_THREADS == 1 || myid() == proc_reloader()
        println("Reload Script...")
        @async revise()
        if AI_THREADS > 1
          waitForRevise()
          resetCounter()
        end
        #println(repeat('\n',100)) # clear console
        println("\33[2J") # clear console: https://rosettacode.org/wiki/Terminal_control/Clear_the_screen#Julia
        if AI_THREADS > 1 sendProcResult(false) end
      end
    end
  end
end
