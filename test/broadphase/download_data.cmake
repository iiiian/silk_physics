function(silk_download_broadphase_archive data_root base_url filename sha256)
    set(archive_dir "${CMAKE_SOURCE_DIR}/build/broadphase-data/archives")
    set(archive "${archive_dir}/${filename}")
    set(stamp "${data_root}/.${filename}.extracted")
    file(MAKE_DIRECTORY "${archive_dir}" "${data_root}")

    set(download TRUE)
    if(EXISTS "${archive}")
        file(SHA256 "${archive}" actual_sha256)
        if("${actual_sha256}" STREQUAL "${sha256}")
            set(download FALSE)
        else()
            file(REMOVE "${archive}")
        endif()
    endif()

    if(download)
        message(STATUS "Downloading broadphase data archive ${filename}")
        file(
            DOWNLOAD "${base_url}/${filename}" "${archive}"
            EXPECTED_HASH "SHA256=${sha256}"
            SHOW_PROGRESS
            STATUS download_status
        )
        list(GET download_status 0 status_code)
        list(GET download_status 1 status_message)
        if(NOT status_code EQUAL 0)
            file(REMOVE "${archive}")
            message(FATAL_ERROR "Unable to download ${filename}: ${status_message}")
        endif()
    endif()

    if(NOT EXISTS "${stamp}")
        message(STATUS "Extracting broadphase data archive ${filename}")
        file(ARCHIVE_EXTRACT INPUT "${archive}" DESTINATION "${data_root}")
        file(TOUCH "${stamp}")
    endif()
endfunction()

function(silk_download_broadphase_data data_root revision)
    set(base_url
        "https://huggingface.co/datasets/Iannn3355/silk_testing/resolve/${revision}"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "armadillo-rollers.tar.gz"
        "27c7fb7349e5498f0652720bbda2b77b4ed6a7fce7797d731a5dad25b4d30037"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "cloth-ball.tar.gz"
        "7f2872c350f1a2176974dad1547a131a0e26a7e6b2c1ef0a3376fe1d84c59442"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "cloth-funnel.tar.gz"
        "0fd949520240d7ad23200266d5b5bcf06d876c71bbe62923a1cd58b1918ba459"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "n-body-simulation.tar.gz"
        "19ec7a4865309daa04c5f81a34bce6b473e6c4c8e1f71bc2241e4f232f5d90ab"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}"
        "puffer-ball-boxes+queries+mma_bool+roots.tar.gz"
        "5b2d06095a065944ca8ba465c9f2f2f3cb6cc12624a8e856c37d185280f89ae3"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "puffer-ball-frames.tar.gz"
        "177a1c6c97c92fd8c394409f60dc68fa8706bc19e1305d9b5ffc44fc75c3cacb"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}"
        "rod-twist-boxes+queries+mma_bool+roots.tar.gz"
        "1e7e24b33a710a01c1d8d26c9591b7fc47d96ec0013f0100a073e4173ef95d52"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "rod-twist-frames-0-999.tar.gz"
        "ed34eae841e14efc347e0b56ab3a5e700c0fd0aef3d21172b760203dde1ebcbb"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "rod-twist-frames-1000-1999.tar.gz"
        "02dd9cd08bb0d2f7177dd9e7338dfdb570970e77d1a2e3963aa4b849bb8e3e1a"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "rod-twist-frames-2000-2999.tar.gz"
        "c0a5308cd4bde1e42d35cbc56dcdb1017a625d74ebc901c8e86c038130c23534"
    )
    silk_download_broadphase_archive(
        "${data_root}" "${base_url}" "rod-twist-frames-3000-4000.tar.gz"
        "bf25bcef3bdbdf29ee25a252f4ea1af3edcab1c08f96f37daabe9b7df7dbbe01"
    )
endfunction()
