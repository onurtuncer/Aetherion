# doc/tikz/CMakeLists.txt

cmake_minimum_required(VERSION 3.16)

# --- Find LaTeX tools ---------------------------------------------------
find_program(LATEXMK_EXECUTABLE latexmk)
find_program(PDFLATEX_EXECUTABLE pdflatex)

if(NOT LATEXMK_EXECUTABLE AND NOT PDFLATEX_EXECUTABLE)
    message(FATAL_ERROR
        "No latexmk or pdflatex found; cannot build TikZ figures.")
endif()

# --- Directories --------------------------------------------------------
set(TIKZ_SRC_DIR   "${CMAKE_CURRENT_SOURCE_DIR}")
set(TIKZ_BUILD_DIR "${CMAKE_CURRENT_BINARY_DIR}")

# Parent directory is doc/
get_filename_component(DOC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

# Destination inside Sphinx static dir
set(TIKZ_DEST_DIR  "${DOC_DIR}/sphinx/_static/figures")

file(MAKE_DIRECTORY "${TIKZ_BUILD_DIR}")
file(MAKE_DIRECTORY "${TIKZ_DEST_DIR}")

# --- Collect all TikZ .tex files ---------------------------------------
file(GLOB TIKZ_TEX_FILES "${TIKZ_SRC_DIR}/*.tex")

set(TIKZ_BUILT_PDFS  "")
set(TIKZ_COPIED_PDFS "")

foreach(tex "${TIKZ_TEX_FILES}")
    get_filename_component(fig_name "${tex}" NAME_WE)

    set(pdf_build "${TIKZ_BUILD_DIR}/${fig_name}.pdf")
    set(pdf_dest  "${TIKZ_DEST_DIR}/${fig_name}.pdf")

    # 1) Build the PDF from the TikZ source
    if(LATEXMK_EXECUTABLE)
        add_custom_command(
            OUTPUT "${pdf_build}"
            COMMAND "${LATEXMK_EXECUTABLE}"
                    -pdf
                    -interaction=nonstopmode
                    -halt-on-error
                    -outdir="${TIKZ_BUILD_DIR}"
                    "${tex}"
            WORKING_DIRECTORY "${TIKZ_BUILD_DIR}"
            DEPENDS "${tex}"
            COMMENT "Building TikZ figure ${fig_name}.pdf with latexmk"
            VERBATIM
        )
    else()
        add_custom_command(
            OUTPUT "${pdf_build}"
            COMMAND "${PDFLATEX_EXECUTABLE}"
                    -interaction=nonstopmode
                    -halt-on-error
                    -output-directory "${TIKZ_BUILD_DIR}"
                    "${tex}"
            WORKING_DIRECTORY "${TIKZ_BUILD_DIR}"
            DEPENDS "${tex}"
            COMMENT "Building TikZ figure ${fig_name}.pdf with pdflatex"
            VERBATIM
        )
    endif()

    list(APPEND TIKZ_BUILT_PDFS "${pdf_build}")

    # 2) Copy built PDF into Sphinx _static/figures
    add_custom_command(
        OUTPUT "${pdf_dest}"
        COMMAND "${CMAKE_COMMAND}" -E copy_if_different
                "${pdf_build}" "${pdf_dest}"
        DEPENDS "${pdf_build}"
        COMMENT "Copying ${fig_name}.pdf to Sphinx static dir"
        VERBATIM
    )

    list(APPEND TIKZ_COPIED_PDFS "${pdf_dest}")
endforeach()

# Aggregate target: build + copy all figures
add_custom_target(tikz_figures ALL
    DEPENDS ${TIKZ_COPIED_PDFS}
)
