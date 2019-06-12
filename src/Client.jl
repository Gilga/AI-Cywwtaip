"""
TESTIFY
"""
module Client

#using Dates

using cywwtaip  # HTW lib from AI professor

####################################################################################################

ROOT = joinpath(@__DIR__,"../")
output=Base.println
pid = 0 # process id
playerNumber = 1
name = nothing
winMsg  = nothing
server = nothing
client = nothing
initalized = false
latency = 0

RuntimeException(msg::String) = ErrorException(msg)
setOutput(poutput) = global output = poutput

getErrorLogFile() = joinpath(ROOT, "error_$(name).log")
getLogFile() = joinpath(ROOT, "out_$(name).log")

outputBacktrace(ex) = open(getErrorLogFile(),"w+") do f Base.showerror(f, ex, catch_backtrace()) end

####################################################################################################

function updateWorld(client::NetworkClient)
  println("START!")
  #while isAlive(client)
    print("#[")
    graphs = world
    #=
    graphs = getGraph(client)
    if length(graphs) > 0
      i=0
      for graph in graphs
        i += 1; j=0;
        for n in graph.neighbors
          j += 1
          node = GraphNode(n)
          if node.owner > 2 println("[$i,$j]: "*toInfoString(node)) end
          if node.owner < 0 println("###### [$i,$j]: "*toInfoString(node)) end
        end
        break
      end
      println("world updated.")
    end
    =#
    print("]# ")
  #end
  println("END!")
end

function main(args::Array{String,1} ;output=Base.println)
  println("--------------------------------------------------------------------------------")
  @info "Start Game..."

  global playerNumber, latency, initalized

  len = length(args)
  if len == 0 throw(RuntimeException("No PID set!")) end # no pid? -> leave

  initalized = false

  #setOutput(output)

  global pid = length(args[1])>0 ? parse(Int,args[1]) : 0
  global server = len>1 ? args[2] : "localhost"
  global name = len>2 ? args[3] : string("Client",pid)
  global winMsg = len>3 ? args[4] : "GEWONNEN!"
  global client = nothing

  try
    cywwtaip.init()
    @info "Connect..."

    # Problem: if connection gets stuck somehow the program has to be restarted...
    # There is currently no function to interrupt a task in julia...
    #t = @async begin client = NetworkClient(server, name, winMsg) end
    #Base.throwto(t, InterruptException())
    #for i=1:9999 if client != nothing break end end
    #if client == nothing return end

    client = NetworkClient(server, name, winMsg)

    global playerNumber = getMyPlayerNumber(client)

    timeLimitBroke = false
    prev_position = nothing
    startTime = time()
    tickTime = 0
    timeLimit = 60 # 60s
    timeLimitBeforeStart = 10 # 60s

    cywwtaip.reset()
    @time world = getGraph(client) # takes a loooong time

    @info "Wait for start..."
    while isAlive(client)
      currentTime = (time() - startTime)

      position = getBotPosition(client, playerNumber)
      if prev_position != nothing && prev_position != position break end
      prev_position = position

      if (time() - tickTime) > 1
        println("Remaining Time($timeLimitBeforeStart): $(timeLimitBeforeStart - round(currentTime))")
        tickTime=time()
      end

      if currentTime > timeLimitBeforeStart timeLimitBroke=true; break end # auto close
    end

    tickTime=0
    startTime = time()-1

    if !timeLimitBroke
      @info "Main Loop..."
      println("\n[START]\n------------------------------")

      while isAlive(client)
        currentTime = (time() - startTime)

        if (time() - tickTime) > 1
          println("Remaining Time($timeLimit): $(timeLimit - round(currentTime))")
          tickTime=time()
        end

        if currentTime > timeLimit timeLimitBroke=true; break end #after Xs break this loop
        #print(".")

        #speed = getBotSpeed(client)
        #score = getScore(client, playerNumber)
        #position = getBotPosition(client, 0, 0)
        #direction = getBotDirection(client, 0)

        #println("Speed: $speed")
        #println("Score: $score")
        #println("Position: $position")
        #println("Direction: $direction")

        #changeMoveDirection(client, 1, -0.08f0)
      end
    end

  catch ex
    outputBacktrace(ex)
    error("NetworkClient error! For more details look in $(getErrorLogFile())")
  end

  print("Game ended.")
  #cywwtaip.destroy() # cleanup cywwtaip
end

end # Client
