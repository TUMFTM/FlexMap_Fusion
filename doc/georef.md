# Georeferencing

- georeferencing of the lanelet2 map is achieved by the inverse application of the [UTM-projection](https://apps.dtic.mil/sti/pdfs/ADA266497.pdf) from the [lanelet2 library](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/tree/master/lanelet2_projection)
- as the calculated transformations during the "Map Alignment" process locally transform the lanelet2 map (and possibly the point cloud map) to the GNSS trajectory, the export with the [UTM-projection](https://apps.dtic.mil/sti/pdfs/ADA266497.pdf) leads to georeferencing