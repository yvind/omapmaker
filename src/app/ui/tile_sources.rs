use eframe::egui;
use walkers::{HttpTiles, MercatorProjection, sources};

#[expect(dead_code)]
#[derive(Debug, Default, Clone, Copy)]
pub enum GoogleServer {
    #[default]
    A = 1,
    B = 2,
    C = 3,
}

pub struct GoogleSatelliteSource(pub GoogleServer);

impl walkers::sources::TileSource for GoogleSatelliteSource {
    type Projection = walkers::MercatorProjection;

    fn tile_url(&self, tile_id: walkers::TileId) -> String {
        format!(
            "https://mt{}.google.com/vt/lyrs=s&x={}&y={}&z={}",
            self.0 as u8, tile_id.x, tile_id.y, tile_id.zoom
        )
    }

    fn attribution(&self) -> sources::Attribution {
        sources::Attribution {
            text: "Google Map Data",
            url: "https://www.google.com/maps/",
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
        HttpTiles::new(GoogleSatelliteSource(GoogleServer::C), ctx.clone()),
    )
}
