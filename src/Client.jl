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

world = GraphNode[]
function ttt(client)
  while isAlive(client)
    sleep(3)
    #global world = getGraph(client)
    print("#")
  end
end

function ready_or_not(channel, wid)
  if !isready(channel)
    println("Process $wid was not ready")
    return nothing
  else
    return take!(channel)
  end
end

function main(args::Array{String,1})
  println("--------------------------------------------------------------------------------")
  @info "Start Game..."

  global playerNumber, latency, initalized

  initalized = false

  len = length(args)
  global server = len>1 ? args[1] : "localhost"
  global name = len>2 ? args[2] : "MyTeam"
  global winMsg = len>3 ? args[3] : "GEWONNEN!"
  global client = nothing
  global world
  serverTimeout = 10

  try
    cywwtaip.init()
    @info "Connect to Server '$server' (Timeout in $serverTimeout seconds)..."
    client = NetworkClient(server, name, winMsg, serverTimeout)

    # a star algorithmn

    global playerNumber = getMyPlayerNumber(client)

    timeLimitBroke = false
    prev_position = nothing
    startTime = time()
    tickTime = 0
    timeLimit = 60 # 60s
    timeLimitBeforeStart = 10 # 60s
    latency = 0

    cywwtaip.reset()
    #@async ttt(client)
    lstart=0
    lend=0

    @info "Wait for start..."
    while isAlive(client)
      currentTime = (time() - startTime)

      lstart = time()
      position = getBotPosition(client, playerNumber)
      lend = time()

      if prev_position != nothing && prev_position != position break end
      prev_position = position

      if (time() - tickTime) > 1
        println("Seconds: $(timeLimitBeforeStart - round(currentTime))")
        tickTime=time()
      end

      if currentTime > timeLimitBeforeStart timeLimitBroke=true; break end # auto close
    end

    latency = lend-lstart
    startTime = lstart-1
    tickTime=0
    world = getGraph(client)

    if !timeLimitBroke
      @info "Main Loop..."
      println("\n[START]\n------------------------------")

      while isAlive(client)
        currentTime = (time() - startTime)

        if (time() - tickTime) > 1
          println("Seconds: $(timeLimit - round(currentTime)), latency: $latency")
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
