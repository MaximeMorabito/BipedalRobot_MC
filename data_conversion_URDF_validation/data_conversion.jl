using LaTeXStrings
using Plots

function toInt16(numbers::Vector{Int})::Vector{Int}
    # First convert it to UInt16 to match the bit size, then reinterpret it as Int16
    new_numbers = reinterpret(Int16, UInt16.(numbers))
    return new_numbers
end

function toInt16(numbers::Vector{Int})::Vector{Int16}
    # We assume `numbers` are 16-bit unsigned integers (UInt16), reinterpret them to Int16
    return reinterpret(Int16, UInt16.(numbers))
end

function transform_position(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to °
    # For the speed, the data is in [ticks]
    # The maximum value is 4095 ticks
    # Hence, to transform the data to °, just multiply it by (360/4095)
    append!(result, numbers[2:end] .*(360 / 4095))

    return result
end

function transform_velocity(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to rad/s
    # The input data is an adimensionalisation of RPMs by 0.229rmp
    # To transform RMP into RPS (rotation per second), divide it by 60
    # Multiply then by 2π to get the rad/s
    append!(result, numbers[2:end] .*0.229*2π / 60) 
    
    return result
end

function transform_PWM(numbers::Vector{Int16}, frequency::Float64)::Vector{Float64}
    # The first element of each line is the index of the data (beginning at 0)
    # Divide this value by the frequency to get time
    result = [numbers[1] / frequency]  
    
    # Transform to PWM [%]
    # A PWM of 100% corresponf to 885 -> we just need to divide what we have by 885
    append!(result, numbers[2:end] .*(1 / 885))

    return result
end

function convert_to_current_basic_model(PWM::Vector{Float64}, RPS::Vector{Float64}):: Vector{Float64}
    # Motor Caracteristics: S. Deligne 
    Un  = 12      # Nominal tension [V]
    kϕ  = 3.6103  # Back-EMF constant ke' [Nm*s/rad] (linked to joint speed)
    R   = 9.3756  # Armature resistance [Ω]
    HGR = 353.5   # Hip gear-ratio
    KGR = 212.6   # Knee gear-ratio
    Kv  = 0.22    # Viscous friction constant [Nm*s/rad] (linked to joint speed)
    τc  = 0.128   # Dry friction torque [Nm] (at the joint level)

    # The first element of each line is the index of the data
    # Divide this value by the frequency to get time

    # Simple motor model : U = R*i + kϕ ̇q -> i = (U - kϕ q̇)/R
    U = PWM[2:end] .* Un
    q̇ = RPS[2:end] ./ [HGR, KGR, HGR, KGR]
    i = (U .- (q̇ .* kϕ)) ./ R

    return append!([PWM[1]],i)
end

function convert_to_torque_basic_model(current::Vector{Float64}, RPS::Vector{Float64}):: Vector{Float64}
    # Motor Caracteristics: S. Deligne
    kϕ  = 3.6103  # Back-EMF constant ke' [Nm*s/rad] (linked to joint speed) 
    HGR = 353.5   # Hip gear-ratio
    KGR = 212.6   # Knee gear-ratio
    Kv  = 0.22    # Viscous friction constant [Nm*s/rad] (linked to joint speed)
    τc  = 0.128   # Dry friction torque [Nm]

    # Simple motor model : τ = kϕ*i - τc - Kv q̇
    q̇ = RPS[2:end] ./ [HGR, KGR, HGR, KGR]
    τ_0 = current[2:end] .* kϕ .- q̇ .* Kv
    τ = ifelse.(τ_0 .> 0, max.(τ_0 .- τc, 0.0), min.(τ_0 .+ τc, 0.0))
    
    return append!([current[1]],τ)
end


    
function process_lines(input_file::String, output_file::String, func::Function, frequency::Float64)
    open(input_file, "r") do infile
        open(output_file, "w") do outfile
            for line in eachline(infile)
                # Replace commas with periods to standardize decimal notation
                line = replace(line, "," => ".")
                # Parse numbers from the line, round to Int, and process
                numbers = round.(Int, parse.(Float64, split(line)))  # Parse and round to integers
                # Transform from Int to Int16 (reinterpret) and then process
                int16_numbers = toInt16(numbers)
                # Apply the transformation function
                processed_numbers = func(int16_numbers, frequency)
                # Write results to output file
                write(outfile, join(processed_numbers, " "), "\n")
            end
        end
    end
end

function convert_data(input_file_1::String, input_file_2::String, output_file::String, func::Function)
    # Open all files
    open(input_file_1, "r") do f1
        open(input_file_2, "r") do f2
            open(output_file, "w") do out
                # Read lines from both input files simultaneously
                for (line1, line2) in zip(eachline(f1), eachline(f2))
                    # Parse floats from the lines
                    data1 = parse.(Float64, split(line1, " "))
                    data2 = parse.(Float64, split(line2, " "))
                    
                    # Apply the operation on the two arrays
                    result = func(data1, data2)
                    
                    # Write the result to the output file
                    write(out, join(result, " "), "\n")
                end
            end
        end
    end
end


# Data files to be preocessed
data_in_position = joinpath(@__DIR__, "..", "data", "Position_20s.txt")
data_in_velocity = joinpath(@__DIR__, "..", "data", "V_20s.txt")
data_in_PWM = joinpath(@__DIR__, "..", "data", "PWM_20s.txt")

# Output data files
data_out_position = joinpath(@__DIR__, "..", "data", "Position.txt")
data_out_velocity = joinpath(@__DIR__, "..", "data", "Velocity.txt")
data_out_PWM = joinpath(@__DIR__, "..", "data", "PWM.txt")
data_out_Current = joinpath(@__DIR__, "..", "data", "Current.txt")
data_out_Torque = joinpath(@__DIR__, "..", "data", "Torque.txt")

# frequency
freq = 20.0

process_lines(data_in_position, data_out_position, transform_position, freq)
process_lines(data_in_velocity, data_out_velocity, transform_velocity, freq)
# Note if you are actually reading the velocity output file :
# The robot first bends its knees, then marks a stop, then walks
# It is thus normal to have values at 0 from ~2.5 - ~5 [s]
process_lines(data_in_PWM, data_out_PWM, transform_PWM, freq)

convert_data(data_out_PWM,data_out_velocity,data_out_Current,convert_to_current_basic_model)
convert_data(data_out_Current,data_out_velocity,data_out_Torque,convert_to_torque_basic_model)


function plot_data(file_path::String, title::String, time_range::Union{Nothing, Tuple{Float64, Float64}} = nothing)
    # Read the file line by line
    lines = readlines(file_path)

    # Parse the data into a matrix
    data = [parse.(Float64, split(line, " ")) for line in lines]

    # Convert the matrix of parsed data into separate columns
    time = [row[1] for row in data]    # First column is time
    data1 = [row[2] for row in data]  # Second column
    data2 = [row[3] for row in data]  # Third column
    data3 = [row[4] for row in data]  # Fourth column
    data4 = [row[5] for row in data]  # Fifth column

    # If time_range is provided, filter the data
    if time_range !== nothing
        # Extract the start and end of the time range
        start_time, end_time = time_range

        # Filter the data based on the time range
        indices = findall(x -> start_time <= x <= end_time, time)
        time = time[indices]
        data1 = data1[indices]
        data2 = data2[indices]
        data3 = data3[indices]
        data4 = data4[indices]
    end

    # Create the plot
    p = plot(time, data1, label="Data 1", xlabel="Time", ylabel="Values", lw=2)
    plot!(p, time, data2, label="Data 2", lw=2)
    plot!(p, time, data3, label="Data 3", lw=2)
    plot!(p, time, data4, label="Data 4", lw=2)

    # Add title
    title!(p, title)

    # Ensure the plot is displayed
    display(p)
end

plot_data(data_out_position, "Position",(5.0,7.0))
#plot_data(data_out_velocity, "Velocity")
#plot_data(data_out_PWM, "PWM")
#plot_data(data_out_Current, "Current")
#plot_data(data_out_Torque, "Torque")

