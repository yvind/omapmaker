use eframe::egui;
use walkers::{HttpTiles, MercatorProjection, sources};

pub struct ArcGisSource;

impl walkers::sources::TileSource for ArcGisSource {
    type Projection = walkers::MercatorProjection;

    fn tile_url(&self, tile_id: walkers::TileId) -> String {
        format!(
            "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{}/{}/{}",
            tile_id.zoom, tile_id.y, tile_id.x
        )
    }

    fn attribution(&self) -> sources::Attribution {
        sources::Attribution {
            text: "nope",
            url: "lol",
            logo_light: None,
            logo_dark: None,
        }
    }

    fn projection(&self) -> Self::Projection {
        MercatorProjection
    }
}

pub fn get_tile_sources(
    ctx: &egui::Context,
) -> (
    HttpTiles<MercatorProjection>,
    HttpTiles<MercatorProjection>,
    HttpTiles<MercatorProjection>,
) {
    (
        HttpTiles::new(sources::OpenStreetMap, ctx.clone()),
        HttpTiles::new(
            sources::OpenTopoMap(sources::OpenTopoServer::C),
            ctx.clone(),
        ),
        HttpTiles::new(ArcGisSource, ctx.clone()),
    )
}
