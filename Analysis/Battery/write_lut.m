function write_lut(filename ,soc_vector, ocv_vector)
    % OCV(soc) Lookup table generation callback
    % args:
    %      soc_vector: state of charge data
    %      ocv_vector : Open circuit voltage corresponding to the soc

    % 2. Open File for Binary Writing (Little-Endian)
    fileID = fopen(filename, 'w', 'ieee-le');
    if fileID == -1
        error('Could not open file for binary writing.');
    end
    
    % 3. Write Metadata Header (Total number of elements as a 32-bit integer)
    num_elements = int32(length(soc_vector));
    fwrite(fileID, num_elements, 'int32'); 
    
    % 4. Write the Breakpoint Vector (SOC) followed by the LUT Data (OCV)
    % Both arrays are written sequentially as 64-bit double precision floats
    fwrite(fileID, soc_vector, 'double');
    fwrite(fileID, ocv_vector, 'double');
    
    % 5. Close File Descriptor
    fclose(fileID);
    disp(['1D Lookup table successfully written to' filename]);
end