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
    
    # transform to °
    # For the speed, the data is in [ticks]
    # The maximum value is 4095 ticks
    # Hence, to transform the data to °, just multiply it by (360/4095)
    append!(result, numbers[2:end] .*(360 / 4095))  # Apply scaling to the rest, starting from the second element
    
    return result
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

# Data files to be preocessed
data_in_position = joinpath(@__DIR__, "..", "data", "Position_20s.txt")
data_in_velocity = joinpath(@__DIR__, "..", "data", "V_20s.txt")
data_in_PWM = joinpath(@__DIR__, "..", "data", "PWM_20s.txt")

# Output data files
data_out_position = joinpath(@__DIR__, "..", "data", "Position.txt")
data_out_velocity = joinpath(@__DIR__, "..", "data", "Velocity.txt")
data_out_PWM = joinpath(@__DIR__, "..", "data", "PWM.txt")

# frequency
freq = 20.0

process_lines(data_in_position, data_out_position, transform_position, freq)
