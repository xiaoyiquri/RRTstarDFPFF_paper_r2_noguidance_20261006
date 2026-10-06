if(NOT EXISTS "D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_noguidance_20261006/build_r2/install_manifest.txt")
  message(FATAL_ERROR "Cannot find install manifest: D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_noguidance_20261006/build_r2/install_manifest.txt")
endif()

file(READ "D:/codex_talk/project_1/project_code/RRTstarDFPFF_paper_r2_noguidance_20261006/build_r2/install_manifest.txt" files)
string(REGEX REPLACE "\n" ";" files "${files}")
foreach(file ${files})
  message(STATUS "Uninstalling $ENV{DESTDIR}${file}")
  if(IS_SYMLINK "$ENV{DESTDIR}${file}" OR EXISTS "$ENV{DESTDIR}${file}")
    exec_program(
      "D:/codex_talk/project_1/tmp/simd-build-tools/cmake/data/bin/cmake.exe" ARGS "-E remove \"$ENV{DESTDIR}${file}\""
      OUTPUT_VARIABLE rm_out
      RETURN_VALUE rm_retval
      )
    if(NOT "${rm_retval}" STREQUAL 0)
      message(FATAL_ERROR "Problem when removing $ENV{DESTDIR}${file}")
    endif()
  else(IS_SYMLINK "$ENV{DESTDIR}${file}" OR EXISTS "$ENV{DESTDIR}${file}")
    message(STATUS "File $ENV{DESTDIR}${file} does not exist.")
  endif()
endforeach()
